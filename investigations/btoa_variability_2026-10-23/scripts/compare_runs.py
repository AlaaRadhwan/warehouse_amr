#!/usr/bin/env python3
"""Cross-run comparison for the experimental (non-smoke) runs: summary table + comparison figures.
usage: compare_runs.py [RUN_ID ...]   (default: every runs/p* directory that has analysis/<run>/metrics.json)"""
import csv, json, sys
from pathlib import Path

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import *

ids = sys.argv[1:] or sorted(p.name for p in (INV / "runs").iterdir() if p.is_dir() and p.name[0] in "pe" and (INV / "analysis" / p.name / "metrics.json").exists())
Ms = {i: json.loads((INV / "analysis" / i / "metrics.json").read_text()) for i in ids}
Rs = {i: Run(INV / "runs" / i) for i in ids}
order = {}
for l in (INV / "runs" / "pilot_order.txt").read_text().splitlines():
    if l and not l.startswith("#"): order[l.split()[0]] = len(order) + 1
g = lambda d, *k: (lambda v: v)(__import__("functools").reduce(lambda a, b: a.get(b) if isinstance(a, dict) else None, k, d))
rows = []
for i, M in Ms.items():
    R, c = Rs[i], M["classification"] or {}; gt = R.gt; m = R.in_goal(gt["t"]); d = np.hypot(gt["x"][m] - GOAL[0], gt["y"][m] - GOAL[1])
    t1 = gt["t"][m][np.argmax(d < 1.0)] - R.tg if np.any(d < 1.0) else None
    pl = M["plans"].get("unsmoothed", {}); fo = pl.get("first", {})
    rows.append({"run": i, "order": order.get(i), "multiplier": M["multiplier"], "outcome": c.get("outcome"), "subtype": c.get("subtype"), "nav2_status": M["client"]["status"],
                 "ended_by": M["client"]["end_reason"], "sim_s": round(M["client"]["sim_duration_s"], 1), "t_within_1m_s": None if t1 is None else round(t1, 1),
                 "gt_min_dist_m": round(c.get("gt_min_dist_m", float("nan")), 3), "gt_final_dist_m": round(c["gt_final"]["dist_xy_m"], 3), "gt_final_yaw_err_deg": round(np.degrees(c["gt_final"]["abs_yaw_err_rad"]), 1),
                 "spin_onset_s": g(c, "flags", "spin_onset_t_goal_s"), "stall_onset_s": g(c, "flags", "stall_onset_t_goal_s"),
                 "first_plan_cusps": fo.get("cusps"), "first_plan_len_m": None if fo.get("length_m") is None else round(fo["length_m"], 1), "first_plan_reverse_m": None if fo.get("reverse_length_m") is None else round(fo["reverse_length_m"], 1),
                 "plan_cusps_min_max": pl.get("cusps_min_max"), "mppi_max_abs_wz": g(M, "commands", "cmd_nav(MPPI)", "max_abs_wz"), "mppi_frac_wz_at_limit": g(M, "commands", "cmd_nav(MPPI)", "frac_wz_at_limit(>=0.99)"),
                 "gt_over_cmd_wz_gain": g(M, "commands", "gt_wz_vs_cmd_final", "gain"), "gt_over_cmd_wz_gain_saturated": g(M, "commands", "gt_wz_vs_cmd_final_saturated", "gain"),
                 "raw_odom_over_gt_wz": g(M, "commands", "raw_odom_wz_vs_gt", "gain_odom_over_gt_median"),
                 "cmd_lag_s": g(M, "commands", "gt_wz_vs_cmd_final", "lag_s"), "mppi_wall_dt_p95_s": g(M, "timing", "mppi_cycle_wall_dt_s", "p95"), "mppi_wall_dt_max_s": g(M, "timing", "mppi_cycle_wall_dt_s", "max"), "mppi_sim_dt_p50_s": g(M, "timing", "mppi_cycle_sim_dt_s", "p50"), "rtf_mean": g(M, "timing", "system_during_goal", "rtf_mean"), "rtf_min": g(M, "timing", "system_during_goal", "rtf_min"),
                 "load1_mean": g(M, "timing", "system_during_goal", "load1_mean"), "ctrl_cpu_mean_pct": g(M, "timing", "system_during_goal", "controller_server_cpu_mean"),
                 "amcl_err_p95_m": g(M, "localization", "amcl_vs_gt", "xy_err_m_p95"), "amcl_err_max_m": g(M, "localization", "amcl_vs_gt", "xy_err_m_max"),
                 "frac_short_transformed": g(M, "association", "frac_short_transformed_plan"), "frac_ambiguous_nearest": g(M, "association", "frac_ambiguous_nearest"),
                 "pred_pos_err_1s_p95_m": g(M, "optimal_vs_actual", "pred_pos_error_1s_m_p95")})
def fmt(v): return "" if v is None else (f"{v:.3g}" if isinstance(v, float) else str(v))
with open(INV / "analysis/summary_table.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); [w.writerow({k: fmt(v) for k, v in r.items()}) for r in rows]
cols = list(rows[0]); md = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)] + ["| " + " | ".join(fmt(r[c]) for c in cols) + " |" for r in rows]
(INV / "analysis/summary_table.md").write_text("\n".join(md) + "\n")
style(plt)
LS = {"SUCCESS": "-", "SPIN": "--", "STALL": ":", "OTHER_FAILURE": "-.", "INCONCLUSIVE": (0, (1, 3))}
# --- cmp 1: distance & cumulative rotation
fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
for i, R in Rs.items():
    c = (Ms[i]["classification"] or {}).get("outcome"); m = R.in_goal(R.gt["t"]); t = R.gt["t"][m] - R.tg
    col, ls = ARM_COL[R.mult], LS.get(c, "-")
    ax[0].plot(t, np.hypot(R.gt["x"][m] - GOAL[0], R.gt["y"][m] - GOAL[1]), color=col, ls=ls, lw=1.6)
    ax[1].plot(t, np.degrees(R.gt["yaw"][m] - R.gt["yaw"][m][0]), color=col, ls=ls, lw=1.6)
    ax[0].text(t[-1], np.hypot(R.gt["x"][m][-1] - GOAL[0], R.gt["y"][m][-1] - GOAL[1]), " " + i.split("_")[0], fontsize=7, color=INK2, va="center")
ax[0].set_ylabel("distance to goal (m)\n[ground truth]"); ax[1].set_ylabel("cumulative yaw rotation (deg)\n[ground truth]"); ax[1].set_xlabel("sim time since goal sent (s)")
h = [plt.Line2D([0], [0], color=ARM_COL[k], lw=2, label=f"wheel_separation_multiplier {k}") for k in ARM_COL] + [plt.Line2D([0], [0], color=INK2, ls=v, label=k.lower().replace("_", " ")) for k, v in LS.items() if any((Ms[i]["classification"] or {}).get("outcome") == k for i in Rs)]
ax[0].legend(handles=h, frameon=False, fontsize=8); [clean(a) for a in ax]
fig.suptitle("All valid runs started at Station B: colour = configuration, line style = classified outcome", x=0.01, ha="left", fontsize=10); fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(INV / "analysis/cmp_distance_rotation.png", dpi=140); plt.close(fig)
# --- cmp 2: first raw global plan of every run (same start/goal)
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
for i, R in Rs.items():
    P = [p for p in R.D["paths"]["unsmoothed"] if R.tg - 0.5 <= p["sim"] <= R.tend]
    if not P: continue
    p = P[0]; c = (Ms[i]["classification"] or {}).get("outcome")
    for a in ax: a.plot(p["x"], p["y"], color=ARM_COL[R.mult], ls=LS.get(c, "-"), lw=1.4, alpha=0.9)
    cu = cusp_indices(p["x"], p["y"], p["yaw"]); [a.plot(p["x"][cu], p["y"][cu], "x", color="#d03b3b", ms=8, mew=2) for a in ax]
ax[1].set_xlim(GOAL[0] - 2, GOAL[0] + 3); ax[1].set_ylim(GOAL[1] - 2.5, GOAL[1] + 2.5)
for a in ax: a.plot(*GOAL[:2], "s", color="#1baf7a", ms=7); a.set_aspect("equal"); clean(a); a.set_xlabel("x (m, map)")
ax[0].set_ylabel("y (m, map)"); ax[0].set_title("First raw State Lattice plan of each run (x = direction change)"); ax[1].set_title("Zoom near the goal")
fig.tight_layout(); fig.savefig(INV / "analysis/cmp_first_plans.png", dpi=140); plt.close(fig)
# --- cmp 3: command-vs-executed gain, MPPI cycle timing, RTF
fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
x = np.arange(len(Rs)); lab = [i.split("_")[0] + "\n" + str(Rs[i].mult) for i in Rs]
def bars(a, key, title, unit, ref=None):
    v = [r.get(key) or 0 for r in rows]; a.bar(x, v, color=[ARM_COL[Rs[i].mult] for i in Rs], width=0.6); a.set_xticks(x); a.set_xticklabels(lab, fontsize=7); a.set_title(title); a.set_ylabel(unit); clean(a)
    if ref is not None: a.axhline(ref, color=INK, lw=0.8, ls="--")
bars(ax[0], "gt_over_cmd_wz_gain", "Executed / commanded yaw rate (|cmd| > 0.2 rad/s)", "ratio (1 = plant follows command)", 1.0); ax[0].set_ylim(0.8, 1.12)
bars(ax[1], "mppi_sim_dt_p50_s", "MPPI cycle interval in SIM time (median)", "sim s (nominal 0.05)", 0.05)
bars(ax[2], "rtf_mean", "Real-time factor during the goal", "sim s per wall s", 1.0)
fig.tight_layout(); fig.savefig(INV / "analysis/cmp_command_timing.png", dpi=140); plt.close(fig)
print((INV / "analysis/summary_table.md").read_text())
