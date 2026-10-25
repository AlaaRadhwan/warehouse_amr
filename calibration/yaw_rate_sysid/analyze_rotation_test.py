#!/usr/bin/env python3
"""Steady-state yaw-rate ratios for a run of run_rotation_test.py.

For each commanded step the analysis window is the constant-command `hold`
phase, minus --settle seconds at the start and --trim at the end.  Inside it
every source's yaw rate is estimated, then expressed as a ratio to (a) the
commanded rate and (b) the Gazebo ground-truth yaw rate.

Estimators
  gt_yaw_slope          LSQ slope of unwrapped ground-truth yaw
  odom_raw_twist        mean twist.angular.z of /diff_drive_controller/odom
  odom_raw_yaw_slope    LSQ slope of the yaw in that odometry's pose
  wheel_vel             r*(w_R - w_L)/(ws*mult) from /joint_states velocities
  wheel_pos_slope       LSQ slope of r*(th_R - th_L)/(ws*mult) from positions
  imu_gyro_z            mean IMU angular_velocity.z
  imu_yaw_slope         LSQ slope of IMU orientation yaw (only if orientation varies)
  fused_twist           mean twist.angular.z of EKF /odom
  fused_yaw_slope       LSQ slope of EKF /odom yaw

`implied_ws` = ws_used * (odometry rate / ground-truth rate): the wheel
separation at which the wheel-based yaw rate would match ground truth,
assuming wheel radius and no-slip.  Informational only; nothing is changed.

Usage: analyze_rotation_test.py RUN_DIR [--settle 2.5] [--trim 0.5] [--no-plot]
"""

import argparse
import json
import math
from pathlib import Path

import csv

import numpy as np

SOURCES = ["gt_yaw_slope", "odom_raw_twist", "odom_raw_yaw_slope", "wheel_vel",
           "wheel_pos_slope", "imu_gyro_z", "imu_yaw_slope", "fused_twist",
           "fused_yaw_slope"]


class Table(dict):
    """Column name -> numpy array (strings stay as object arrays)."""

    def __len__(self):
        return len(next(iter(self.values()))) if dict.__len__(self) else 0

    def where(self, m):
        return Table({k: v[m] for k, v in self.items()})


def read_csv(path):
    with open(path, newline="") as f:
        rd = csv.reader(f)
        head = next(rd)
        cols = list(zip(*rd)) or [()] * len(head)
    t = Table()
    for h, c in zip(head, cols):
        try:
            t[h] = np.array(c, dtype=float)
        except ValueError:
            t[h] = np.array(c, dtype=object)
    return t


def yaw(df):
    return np.arctan2(2 * (df["qw"] * df["qz"] + df["qx"] * df["qy"]),
                      1 - 2 * (df["qy"] ** 2 + df["qz"] ** 2))


def tcol(df):
    """Prefer header stamps (sensor time); fall back to receive time."""
    ts = df.get("t_stamp")
    if ts is not None and len(ts) > 1 and (ts > 0).all() and (np.diff(ts) >= 0).all():
        return ts
    return df["t_recv"]


def slope(t, y):
    """LSQ slope and its standard error."""
    if len(t) < 5:
        return math.nan, math.nan
    A = np.vstack([t - t.mean(), np.ones_like(t)]).T
    (m, _), res, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ np.array([m, y.mean()])
    se = math.sqrt(resid @ resid / (len(t) - 2) / np.sum((t - t.mean()) ** 2))
    return float(m), se


def mean_sem(x):
    if len(x) < 5:
        return math.nan, math.nan
    return float(np.mean(x)), float(np.std(x, ddof=1) / math.sqrt(len(x)))


def load(run):
    d = {}
    for k in ("cmd", "cmd_out", "gt", "odom_raw", "joints", "imu", "odom_fused"):
        d[k] = read_csv(run / f"{k}.csv")
    return d


def window(df, t, lo, hi):
    m = (t >= lo) & (t <= hi)
    return df.where(m), t[m]


def estimate(d, lo, hi, ws_used, r_l, r_r):
    out = {}

    def add(name, val, se, n):
        out[name] = (val, se, n)

    g = d["gt"]; tg = tcol(g)
    w, tw = window(g, tg, lo, hi)
    if len(w) > 5:
        psi = np.unwrap(yaw(w))
        add("gt_yaw_slope", *slope(tw, psi), len(w))

    for key, tw_name, sl_name in (("odom_raw", "odom_raw_twist", "odom_raw_yaw_slope"),
                                  ("odom_fused", "fused_twist", "fused_yaw_slope")):
        o = d[key]; to = tcol(o)
        w, tw = window(o, to, lo, hi)
        if len(w) > 5:
            add(tw_name, *mean_sem(w["wz"]), len(w))
            add(sl_name, *slope(tw, np.unwrap(yaw(w))), len(w))

    j = d["joints"]; tj = tcol(j)
    w, tw = window(j, tj, lo, hi)
    if len(w) > 5:
        wv = r_r * w["right_vel"] - r_l * w["left_vel"]  # = ws * wz
        add("wheel_vel", *[v / ws_used for v in mean_sem(wv)], len(w))
        wp = (r_r * w["right_pos"] - r_l * w["left_pos"]) / ws_used
        add("wheel_pos_slope", *slope(tw, wp), len(w))

    i = d["imu"]; ti = tcol(i)
    w, tw = window(i, ti, lo, hi)
    if len(w) > 5:
        add("imu_gyro_z", *mean_sem(w["wz"]), len(w))
        psi = np.unwrap(yaw(w))
        if np.ptp(psi) > 0.05:  # orientation actually evolves
            add("imu_yaw_slope", *slope(tw, psi), len(w))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--settle", type=float, default=2.5,
                    help="seconds to skip after the command reaches steady value")
    ap.add_argument("--trim", type=float, default=0.5, help="seconds to drop at hold end")
    ap.add_argument("--ws", type=float, default=None,
                    help="override effective wheel separation used by the controller")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    run = Path(args.run_dir)
    meta = json.loads((run / "metadata.json").read_text())
    p = meta.get("controller_params") or {}
    ws_used = args.ws or (p.get("wheel_separation") or 0.69) * (p.get("wheel_separation_multiplier") or 1.0)
    r = p.get("wheel_radius") or 0.10
    r_l = r * (p.get("left_wheel_radius_multiplier") or 1.0)
    r_r = r * (p.get("right_wheel_radius_multiplier") or 1.0)
    d = load(run)

    cmd = d["cmd"]
    rows = []
    hold = cmd["phase"] == "hold"
    for idx in sorted(set(cmd["step"][hold])):
        m = hold & (cmd["step"] == idx)
        t_h = cmd["t"][m]
        target = float(cmd["target_wz"][m][0])
        lo, hi = t_h.min() + args.settle, t_h.max() - args.trim
        est = estimate(d, lo, hi, ws_used, r_l, r_r)
        if "gt_yaw_slope" not in est:
            continue
        gt = est["gt_yaw_slope"][0]
        co = d["cmd_out"]["wz"][(d["cmd_out"]["t_recv"] >= lo) & (d["cmd_out"]["t_recv"] <= hi)]
        row = {"step": int(idx), "target": target, "t0": lo, "t1": hi,
               "cmd_out_mean": float(co.mean()) if len(co) else math.nan}
        for s_ in SOURCES:
            if s_ not in est:
                continue
            v, se, n = est[s_]
            row[s_] = v
            row[f"{s_}_se"] = se
            row[f"{s_}_n"] = n
            row[f"{s_}/cmd"] = v / target
            row[f"{s_}/gt"] = v / gt if gt else math.nan
        if "odom_raw_yaw_slope" in est:
            row["implied_ws_raw_odom"] = ws_used * est["odom_raw_yaw_slope"][0] / gt
        if "wheel_pos_slope" in est:
            row["implied_ws_wheel_pos"] = ws_used * est["wheel_pos_slope"][0] / gt
        rows.append(row)

    if not rows:
        raise SystemExit("no hold windows with ground-truth data found")
    keys = []
    for rw in rows:
        keys += [k for k in rw if k not in keys]
    with open(run / "yaw_rate_summary.csv", "w", newline="") as f:
        w_ = csv.DictWriter(f, fieldnames=keys)
        w_.writeheader()
        w_.writerows(rows)

    def has(k):
        return any(k in rw for rw in rows)

    def fmt(rw, k, f="{:.3f}"):
        v = rw.get(k)
        return "-" if v is None or math.isnan(v) else f.format(v)

    def mean(vals):
        vals = [v for v in vals if v is not None and not math.isnan(v)]
        return sum(vals) / len(vals) if vals else math.nan

    # ---------------------------------------------------------------- report
    L = [f"# Yaw-rate system identification: {run.name}", "",
         f"Status: {meta.get('status')}  |  ws used by controller: {ws_used:.4f} m  "
         f"|  wheel r (L/R): {r_l:.4f}/{r_r:.4f}",
         f"open_loop={p.get('open_loop')}  position_feedback={p.get('position_feedback')}  "
         f"window: hold minus {args.settle}s start / {args.trim}s end", ""]

    def table(title, suffix):
        L.append(f"## {title}")
        cols = [c for c in SOURCES if has(f"{c}{suffix}") and not (suffix == "/gt" and c == "gt_yaw_slope")]
        L.append("| cmd [rad/s] | " + " | ".join(cols) + " |")
        L.append("|---|" + "---|" * len(cols))
        for rw in rows:
            L.append(f"| {rw['target']:+.2f} | " + " | ".join(fmt(rw, f"{c}{suffix}") for c in cols) + " |")
        L.append("")

    table("Ratio to COMMAND (source yaw rate / commanded)", "/cmd")
    table("Ratio to GROUND TRUTH (source yaw rate / Gazebo yaw rate)", "/gt")
    L.append("## Absolute rates [rad/s]")
    cols = [c for c in SOURCES if has(c)]
    L.append("| cmd | cmd_out | " + " | ".join(cols) + " |")
    L.append("|---|---|" + "---|" * len(cols))
    for rw in rows:
        L.append(f"| {rw['target']:+.2f} | {fmt(rw, 'cmd_out_mean', '{:+.4f}')} | " +
                 " | ".join(fmt(rw, c, "{:+.4f}") for c in cols) + " |")
    L.append("")

    L.append("## Speed / sign dependence (ratio to ground truth; gt/cmd is the true tracking ratio)")
    ratio_cols = [f"{c}/gt" for c in SOURCES if has(f"{c}/gt") and c != "gt_yaw_slope"]
    L.append("| |cmd| | gt/cmd (+) | gt/cmd (-) | " + " | ".join(c.replace('/gt', '') for c in ratio_cols) + " |")
    L.append("|---|---|---|" + "---|" * len(ratio_cols))
    for mag in sorted({abs(rw["target"]) for rw in rows}):
        g = [rw for rw in rows if abs(rw["target"]) == mag]
        gp = mean([rw["gt_yaw_slope/cmd"] for rw in g if rw["target"] > 0])
        gn = mean([rw["gt_yaw_slope/cmd"] for rw in g if rw["target"] < 0])
        L.append(f"| {mag:.2f} | {gp:.3f} | {gn:.3f} | " +
                 " | ".join(f"{mean([rw.get(c) for rw in g]):.3f}" for c in ratio_cols) + " |")
    L.append("")

    imp = [c for c in ("implied_ws_raw_odom", "implied_ws_wheel_pos") if has(c)]
    if imp:
        L.append("## Implied effective wheel separation [m] (informational; NOT applied)")
        L.append("| cmd | " + " | ".join(imp) + " |")
        L.append("|---|" + "---|" * len(imp))
        for rw in rows:
            L.append(f"| {rw['target']:+.2f} | " + " | ".join(fmt(rw, c, "{:.4f}") for c in imp) + " |")
        L.append("\nMean over steps: " + ", ".join(f"{c}={mean([rw.get(c) for rw in rows]):.4f}" for c in imp))
        L.append("")

    L.append("## Data checks")
    L.append(f"sample counts: {meta.get('sample_counts')}")
    L.append("Per-estimate standard errors and sample counts are in `yaw_rate_summary.csv`.")
    (run / "report.md").write_text("\n".join(L))
    print("\n".join(L))

    if not args.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        cols = [c for c in SOURCES if has(f"{c}/gt") and c != "gt_yaw_slope"]
        fig, ax = plt.subplots(2, 1, figsize=(10, 8))
        x = np.arange(len(rows)); wdt = 0.8 / max(len(cols), 1)
        for k, c in enumerate(cols):
            ax[0].bar(x + k * wdt, [rw.get(f"{c}/gt", math.nan) for rw in rows], wdt, label=c)
        ax[0].axhline(1.0, color="k", lw=0.8)
        ax[0].set_xticks(x + 0.4 - wdt / 2)
        ax[0].set_xticklabels([f"{rw['target']:+.1f}" for rw in rows])
        ax[0].set_xlabel("commanded yaw rate [rad/s]"); ax[0].set_ylabel("source / ground truth")
        ax[0].legend(fontsize=7, ncol=3)
        for key, label in (("gt", "ground truth yaw"), ("odom_raw", "raw odom yaw"),
                           ("odom_fused", "fused yaw")):
            ax[1].plot(tcol(d[key]), np.unwrap(yaw(d[key])), label=label)
        ax[1].set_xlabel("sim time [s]"); ax[1].set_ylabel("unwrapped yaw [rad]"); ax[1].legend()
        fig.tight_layout(); fig.savefig(run / "yaw_rate_ratios.png", dpi=120)


if __name__ == "__main__":
    main()
