#!/usr/bin/env python3
"""Steady-state longitudinal velocity ratios for a run_rotation_test.py --mode linear run.

Per hold window (hold minus --settle at start / --trim at end):
  gt_long / gt_lat   Gazebo ground-truth world velocity (LSQ slope of x, y)
                     projected on the mean body heading
  gt_yaw_rate        LSQ slope of unwrapped ground-truth yaw (unintended rotation)
  wheel_vel          nominal-radius wheel speed  (r_L*w_L + r_R*w_R)/2 from /joint_states
  wheel_pos_slope    same from wheel positions
  odom_raw_twist / odom_raw_pos   /diff_drive_controller/odom twist.linear.x / path speed
  fused_twist / fused_pos         EKF /odom
  cmd_out            controller-limited command
  imu_wz, imu_ax     mean gyro z / body-x accel (steady state: ~0)

k_v = gt_long / wheel_vel  (actual / nominal-model longitudinal velocity).

Usage: analyze_linear_test.py RUN_DIR [--settle 2.5] [--trim 0.5] [--no-plot]
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from analyze_rotation_test import read_csv, slope, tcol, yaw, mean_sem


def win(tab, t, lo, hi):
    return tab.where((t >= lo) & (t <= hi)), t[(t >= lo) & (t <= hi)]


def path_speed(tab, t, psi):
    """Slope of x,y projected on heading psi -> (long, lat)."""
    vx, _ = slope(t, tab["x"])
    vy, _ = slope(t, tab["y"])
    return (vx * math.cos(psi) + vy * math.sin(psi),
            -vx * math.sin(psi) + vy * math.cos(psi))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--settle", type=float, default=2.5)
    ap.add_argument("--trim", type=float, default=0.5)
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    run = Path(a.run_dir)
    meta = json.loads((run / "metadata.json").read_text())
    p = meta.get("controller_params") or {}
    r = p.get("wheel_radius") or 0.10
    r_l = r * (p.get("left_wheel_radius_multiplier") or 1.0)
    r_r = r * (p.get("right_wheel_radius_multiplier") or 1.0)
    d = {k: read_csv(run / f"{k}.csv") for k in
         ("cmd", "cmd_out", "gt", "odom_raw", "joints", "imu", "odom_fused")}
    cmd = d["cmd"]
    hold = cmd["phase"] == "hold"
    rows = []
    for idx in sorted(set(cmd["step"][hold])):
        m = hold & (cmd["step"] == idx)
        target = float(cmd["target_v"][m][0])
        lo, hi = cmd["t"][m].min() + a.settle, cmd["t"][m].max() - a.trim
        row = {"step": int(idx), "target": target, "t0": lo, "t1": hi}

        g = d["gt"]; w, tw = win(g, tcol(g), lo, hi)
        psi_arr = np.unwrap(yaw(w))
        psi = float(psi_arr.mean())
        row["gt_long"], row["gt_lat"] = path_speed(w, tw, psi)
        row["gt_yaw_rate"], _ = slope(tw, psi_arr)
        row["gt_long_n"] = len(w)
        row["gt_lat_dev_m"] = row["gt_lat"] * (hi - lo)

        co = d["cmd_out"]["wz"]  # placeholder to keep column names explicit below
        c = d["cmd_out"]; mm = (c["t_recv"] >= lo) & (c["t_recv"] <= hi)
        row["cmd_out"] = float(c["vx"][mm].mean()) if mm.any() else math.nan

        j = d["joints"]; w, tw = win(j, tcol(j), lo, hi)
        wl, wr = w["left_vel"], w["right_vel"]
        row["wheel_vel"], row["wheel_vel_se"] = mean_sem((r_l * wl + r_r * wr) / 2)
        row["wheel_pos_slope"], _ = slope(tw, (r_l * w["left_pos"] + r_r * w["right_pos"]) / 2)
        row["wL_rad_s"], row["wR_rad_s"] = float(wl.mean()), float(wr.mean())
        row["wheel_asym"] = (row["wR_rad_s"] - row["wL_rad_s"]) / (
            (row["wR_rad_s"] + row["wL_rad_s"]) / 2)

        for key, tn, pn in (("odom_raw", "odom_raw_twist", "odom_raw_pos"),
                            ("odom_fused", "fused_twist", "fused_pos")):
            o = d[key]; w, tw = win(o, tcol(o), lo, hi)
            if len(w) > 5:
                row[tn] = float(w["vx"].mean())
                row[pn] = path_speed(w, tw, float(np.unwrap(yaw(w)).mean()))[0]
                if key == "odom_raw":
                    row["odom_raw_wz"] = float(w["wz"].mean())
                else:
                    row["fused_wz"] = float(w["wz"].mean())

        i = d["imu"]; w, _ = win(i, tcol(i), lo, hi)
        row["imu_wz"], row["imu_ax"] = float(w["wz"].mean()), float(w["ax"].mean())

        gtl = row["gt_long"]
        row["k_v"] = gtl / row["wheel_vel"]
        row["gt/cmd"] = gtl / target
        for s in ("cmd_out", "wheel_vel", "wheel_pos_slope", "odom_raw_twist",
                  "odom_raw_pos", "fused_twist", "fused_pos"):
            if s in row:
                row[f"{s}/cmd"] = row[s] / target
                row[f"{s}/gt"] = row[s] / gtl
        rows.append(row)

    keys = []
    for rw in rows:
        keys += [k for k in rw if k not in keys]
    with open(run / "linear_summary.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=keys); wr.writeheader(); wr.writerows(rows)

    # whole-run net drift (ground truth)
    g = d["gt"]
    net = {"dx": g["x"][-1] - g["x"][0], "dy": g["y"][-1] - g["y"][0],
           "dyaw_rad": float(np.unwrap(yaw(g))[-1] - np.unwrap(yaw(g))[0])}

    def f(rw, k, fmt="{:.4f}"):
        v = rw.get(k)
        return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else fmt.format(v)

    L = [f"# Straight-line system identification: {run.name}", "",
         f"Status: {meta.get('status')} | controller r (L/R) = {r_l}/{r_r} m | "
         f"ws = {p.get('wheel_separation')} | open_loop={p.get('open_loop')} "
         f"position_feedback={p.get('position_feedback')}",
         f"Window: hold minus {a.settle}s start / {a.trim}s end", "",
         "## Longitudinal velocity [m/s]",
         "| cmd | cmd_out | GT long | wheel (nominal r) | wheel pos | raw odom twist | raw odom pos | fused twist | fused pos |",
         "|---|---|---|---|---|---|---|---|---|"]
    for rw in rows:
        L.append(f"| {rw['target']:+.2f} | " + " | ".join(
            f(rw, k, "{:+.4f}") for k in ("cmd_out", "gt_long", "wheel_vel", "wheel_pos_slope",
                                          "odom_raw_twist", "odom_raw_pos", "fused_twist", "fused_pos")) + " |")
    L += ["", "## Ratios to command", "| cmd | GT | cmd_out | wheel | raw odom twist | fused twist |",
          "|---|---|---|---|---|---|"]
    for rw in rows:
        L.append(f"| {rw['target']:+.2f} | " + " | ".join(
            f(rw, k, "{:.4f}") for k in ("gt/cmd", "cmd_out/cmd", "wheel_vel/cmd",
                                         "odom_raw_twist/cmd", "fused_twist/cmd")) + " |")
    L += ["", "## k_v = GT longitudinal / nominal-radius wheel velocity",
          "| cmd | k_v | raw odom/GT | fused/GT | wheel L [rad/s] | wheel R [rad/s] | (wR-wL)/mean |",
          "|---|---|---|---|---|---|---|"]
    for rw in rows:
        L.append(f"| {rw['target']:+.2f} | {rw['k_v']:.5f} | {f(rw,'odom_raw_twist/gt')} | "
                 f"{f(rw,'fused_twist/gt')} | {rw['wL_rad_s']:+.4f} | {rw['wR_rad_s']:+.4f} | "
                 f"{rw['wheel_asym']:+.5f} |")
    kv = np.array([rw["k_v"] for rw in rows])
    L += ["", f"k_v mean = {kv.mean():.5f}, std = {kv.std(ddof=1) if len(kv) > 1 else 0:.5f}, "
          f"min/max = {kv.min():.5f}/{kv.max():.5f}", "",
          "## Unintended drift in steady-state windows (ground truth + sensors)",
          "| cmd | GT lateral vel [m/s] | lateral dev over window [m] | GT yaw rate [rad/s] | IMU wz | raw odom wz | fused wz | IMU ax |",
          "|---|---|---|---|---|---|---|---|"]
    for rw in rows:
        L.append(f"| {rw['target']:+.2f} | {rw['gt_lat']:+.5f} | {rw['gt_lat_dev_m']:+.4f} | "
                 f"{rw['gt_yaw_rate']:+.5f} | {rw['imu_wz']:+.5f} | {f(rw,'odom_raw_wz','{:+.5f}')} | "
                 f"{f(rw,'fused_wz','{:+.5f}')} | {rw['imu_ax']:+.5f} |")
    L += ["", f"Whole-run net ground-truth displacement: dx={net['dx']:+.3f} m, dy={net['dy']:+.3f} m, "
          f"dyaw={net['dyaw_rad']:+.4f} rad", "",
          f"sample counts: {meta.get('sample_counts')}"]
    (run / "linear_report.md").write_text("\n".join(L))
    print("\n".join(L))

    if not a.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))
        x = np.arange(len(rows)); lab = [f"{rw['target']:+.1f}" for rw in rows]
        srcs = [("wheel_vel", "wheel (nominal r)"), ("odom_raw_twist", "raw odom"),
                ("fused_twist", "fused odom"), ("cmd_out", "controller out")]
        for k, (s, n) in enumerate(srcs):
            ax[0, 0].bar(x + k * 0.2, [rw.get(f"{s}/gt", np.nan) for rw in rows], 0.2, label=n)
        ax[0, 0].axhline(1, color="k", lw=.8); ax[0, 0].set_xticks(x + .3); ax[0, 0].set_xticklabels(lab)
        ax[0, 0].set_title("source / ground-truth longitudinal velocity"); ax[0, 0].legend(fontsize=7)
        ax[0, 1].bar(x, kv); ax[0, 1].set_xticks(x); ax[0, 1].set_xticklabels(lab)
        ax[0, 1].set_ylim(min(kv.min(), 1) * .99, max(kv.max(), 1) * 1.01); ax[0, 1].axhline(1, color="k", lw=.8)
        ax[0, 1].set_title("k_v = GT / nominal wheel velocity")
        ax[1, 0].plot(g["x"] - g["x"][0], g["y"] - g["y"][0]); ax[1, 0].set_aspect("equal", "datalim")
        ax[1, 0].set_title("ground-truth path [m]"); ax[1, 0].set_xlabel("x"); ax[1, 0].set_ylabel("y")
        ax[1, 1].bar(x - .2, [rw["gt_lat"] * 1e3 for rw in rows], .4, label="GT lateral vel [mm/s]")
        ax[1, 1].bar(x + .2, [rw["gt_yaw_rate"] * 1e3 for rw in rows], .4, label="GT yaw rate [mrad/s]")
        ax[1, 1].set_xticks(x); ax[1, 1].set_xticklabels(lab); ax[1, 1].legend(fontsize=7)
        ax[1, 1].set_title("unintended drift")
        fig.tight_layout(); fig.savefig(run / "linear_ratios.png", dpi=120)


if __name__ == "__main__":
    main()
