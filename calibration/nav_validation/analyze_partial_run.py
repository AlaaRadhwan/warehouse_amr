#!/usr/bin/env python3
"""Analyze a B->A run that was stopped by the operator (no client result file).
Uses the bag + gt.csv only. Usage: analyze_partial_run.py RUN_DIR"""
import csv, json, math, sys
from pathlib import Path
import numpy as np
from analyze_b_to_a import read_bag, series_twist, series_odom, wrap, GOAL
run = Path(sys.argv[1])
msgs = read_bag(run / "bag")
t_goal = msgs["/unsmoothed_plan"][0][0]
gt = np.array([[float(x) for x in r] for r in list(csv.reader(open(run / "gt.csv")))[1:]])
t = gt[:, 1]; keep = t >= t_goal
t, x, y = t[keep], gt[keep, 2], gt[keep, 3]
yaw = np.unwrap([math.atan2(2*(r[8]*r[7]+r[5]*r[6]), 1-2*(r[6]**2+r[7]**2)) for r in gt[keep]])
d = np.hypot(x-GOAL[0], y-GOAL[1])
k = 25; w = (yaw[k:]-yaw[:-k])/(t[k:]-t[:-k]); tw = 0.5*(t[k:]+t[:-k])
cmd = series_twist(msgs, "/cmd_vel"); nav = series_twist(msgs, "/cmd_vel_nav")
raw = series_odom(msgs, "/diff_drive_controller/odom"); fus = series_odom(msgs, "/odom")
imu = np.array([[tt, m.angular_velocity.z] for tt, m in msgs["/imu/data_raw"]])
def ratio(a, c):
    ww = np.interp(a[:, 0], tw, w, left=np.nan, right=np.nan); mk = (np.abs(ww) > 0.2) & np.isfinite(ww)
    return float(np.mean(a[mk, c]/ww[mk])) if mk.sum() > 10 else None
out = dict(sim_s_recorded_after_goal=float(t[-1]-t_goal), min_dist_to_goal=float(d.min()),
           final_dist=float(d[-1]), final_yaw_err_deg=float(np.degrees(wrap(yaw[-1]-GOAL[2]))),
           yaw_rate_ratio_to_gt=dict(raw_wheel_odom=ratio(raw, 2), fused=ratio(fus, 2), imu=ratio(imu, 1), cmd_vel=ratio(cmd, 2)),
           cmd_wz_abs_max=float(np.abs(cmd[:, 2]).max()), nav_wz_abs_max=float(np.abs(nav[:, 2]).max()))
# phases: time within 1.0 / 1.5 / 3 m, and when circling starts (cum rotation per distance band)
for R in (1.0, 1.5, 3.0):
    m = d < R
    out[f"within_{R}m"] = dict(time_s=float(np.sum(np.diff(t, prepend=t[0])[m])),
        total_rotation_deg=float(np.degrees(np.sum(np.abs(np.diff(yaw[m]))))) if m.sum() > 1 else 0.0,
        max_abs_gt_wz=float(np.abs(np.interp(t[m], tw, w)).max()) if m.sum() > 1 else 0.0)
# sustained spin: longest run with |wz|>0.5 and the stretch with |cmd wz| pinned at limit
big = np.abs(w) > 0.5; best = cur = 0.0
for i in range(1, len(tw)):
    cur = cur + (tw[i]-tw[i-1]) if big[i] else 0.0; best = max(best, cur)
out["longest_continuous_|gt_wz|>0.5_s"] = best
out["time_|cmd_wz|>0.9_s"] = float(np.sum(np.diff(cmd[:, 0], prepend=cmd[0, 0])[np.abs(cmd[:, 2]) > 0.9]))
out["total_rotation_deg_whole_run"] = float(np.degrees(np.sum(np.abs(np.diff(yaw)))))
# final 30 s
f = tw > tw[-1]-30
out["last_30s"] = dict(mean_gt_wz=float(w[f].mean()), mean_abs_cmd_wz=float(np.abs(cmd[cmd[:, 0] > t[-1]-30, 2]).mean()),
                       dist_range_m=[float(d[t > t[-1]-30].min()), float(d[t > t[-1]-30].max())])
(run / "partial_run_metrics.json").write_text(json.dumps(out, indent=2)); print(json.dumps(out, indent=1))
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
ax[0].plot(x, y); ax[0].plot(*GOAL[:2], "r*", ms=12); ax[0].set_aspect("equal", "datalim"); ax[0].set_title("ground-truth path")
ax[1].plot(t-t_goal, d); ax[1].set_title("distance to goal [m]"); ax[1].set_xlabel("sim s")
ax[2].plot(tw-t_goal, w, "k", label="GT"); ax[2].plot(cmd[:, 0]-t_goal, cmd[:, 2], lw=.7, label="cmd_vel"); ax[2].plot(imu[:, 0]-t_goal, imu[:, 1], lw=.6, label="IMU")
ax[2].plot(raw[:, 0]-t_goal, raw[:, 2], lw=.6, label="raw odom"); ax[2].legend(fontsize=7); ax[2].set_title("yaw rate [rad/s]"); ax[2].set_xlabel("sim s")
fig.tight_layout(); fig.savefig(run / "partial_run_overview.png", dpi=110)
