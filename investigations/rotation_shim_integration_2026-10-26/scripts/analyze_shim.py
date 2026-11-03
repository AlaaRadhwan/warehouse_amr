#!/usr/bin/env python3
"""Offline analysis of one Rotation Shim check run (reads the small bag, gt.csv, client_result.json; writes analysis/<run>/).

Who produced a /cmd_vel_nav sample?  MPPI publishes /optimal_trajectory inside every computeVelocityCommands call
(nav2_mppi_controller 1.3.12 controller.cpp:103-116, visualize: true), just before controller_server publishes the command.
Measured in run s02: the /cmd_vel_nav sample arrives 0.1 ms (p99 0.7 ms) after its /optimal_trajectory in bag (wall) time.
So: a /cmd_vel_nav sample is MPPI's iff an /optimal_trajectory arrived within the 5 ms before it, otherwise the shim made it.
(Header stamps cannot be matched: the controller loop runs on wall time, ~0.02-0.025 sim-s apart, and the two stamps differ.)
This is an inference from source + timing data, not a published signal.
usage: analyze_shim.py RUN_DIR
"""
import csv, json, math, sys
from pathlib import Path

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run = Path(sys.argv[1]); out = Path(run.parent.parent / "analysis" / run.name); out.mkdir(parents=True, exist_ok=True)
yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
XY_TOL, YAW_TOL = 0.25, 0.20
try:   # tolerances actually loaded in this run
    import yaml
    _g = yaml.safe_load((run / "param_dump/_controller_server.yaml").read_text())["/controller_server"]["ros__parameters"]["general_goal_checker"]
    XY_TOL, YAW_TOL = float(_g["xy_goal_tolerance"]), float(_g["yaw_goal_tolerance"])
except Exception as e:
    print("WARNING: using default tolerances:", e)

r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=str(run / "bag"), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
nav, smo, cmd, odom, opt, plans, tf_odom, amcl = [], [], [], [], set(), [], [], []
while r.has_next():
    t, raw, tns = r.read_next(); m = deserialize_message(raw, types[t])
    if t == "/cmd_vel_nav": nav.append((st(m.header), m.twist.linear.x, m.twist.angular.z, tns * 1e-9))
    elif t == "/cmd_vel_smoothed": smo.append((st(m.header), m.twist.linear.x, m.twist.angular.z))
    elif t == "/cmd_vel": cmd.append((st(m.header), m.twist.linear.x, m.twist.angular.z))
    elif t == "/diff_drive_controller/odom": odom.append((st(m.header), m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif t == "/optimal_trajectory": opt.add(tns * 1e-9)
    elif t == "/amcl_pose": amcl.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yawq(m.pose.pose.orientation)))
    elif t == "/plan":
        p = m.poses
        plans.append({"sim": st(m.header), "x": np.array([q.pose.position.x for q in p]), "y": np.array([q.pose.position.y for q in p]),
                      "yaw": np.array([yawq(q.pose.orientation) for q in p])})
nav, smo, cmd, odom, amcl = (np.array(a) for a in (nav, smo, cmd, odom, amcl))
opt = np.array(sorted(opt)); j = np.searchsorted(opt, nav[:, 3]); prev = opt[np.clip(j - 1, 0, len(opt) - 1)]
src_mppi = (nav[:, 3] - prev >= -0.001) & (nav[:, 3] - prev <= 0.005)
gt = np.array([[float(c) for c in row] for row in list(csv.reader(open(run / "gt.csv")))[1:]])      # wall sim x y z qx qy qz qw
gt_t, gt_x, gt_y = gt[:, 1], gt[:, 2], gt[:, 3 - 0]
gt_x, gt_y = gt[:, 2], gt[:, 3]; gt_yaw = np.unwrap(2 * np.arctan2(gt[:, 7], gt[:, 8]))
G = lambda t, a: np.interp(t, gt_t, a)
res = json.loads((run / "client_result.json").read_text())

def cusps_reverse(p):
    """Reverse length and cusps of a lattice path: pose yaw vs direction of travel (yaw is the vehicle heading)."""
    dx, dy = np.diff(p["x"]), np.diff(p["y"]); seg = np.hypot(dx, dy); ok = seg > 1e-6
    fwd = (np.cos(p["yaw"][:-1]) * dx + np.sin(p["yaw"][:-1]) * dy) > 0
    d = fwd[ok]; return float(seg[ok][~d].sum()), int((d[1:] != d[:-1]).sum())

rep = []
for gr in res["goals"]:
    nm, (gx, gy, gyaw) = gr["name"], gr["goal"]
    if "sim_accepted" not in gr: rep.append(f"## {nm}: {gr['status']}"); continue
    t0, t1 = gr["sim_accepted"], gr["sim_finished"]
    k = (nav[:, 0] >= t0) & (nav[:, 0] <= t1 + 0.01); T, VX, WZ, M = nav[k, 0], nav[k, 1], nav[k, 2], src_mppi[k]
    dist = np.hypot(G(T, gt_x) - gx, G(T, gt_y) - gy); yaw = G(T, gt_yaw); yerr = wrap(yaw - gyaw)
    # phases = contiguous runs of the same source
    ph = []; s = 0
    for i in range(1, len(T) + 1):
        if i == len(T) or M[i] != M[s]:
            ph.append(("MPPI" if M[s] else "SHIM", T[s] - t0, T[i - 1] - t0, float(yaw[i - 1] - yaw[s]), float(dist[s]), float(dist[i - 1]),
                       float(np.max(np.abs(WZ[s:i]))), float(np.max(np.abs(VX[s:i]))), i - s)); s = i
    pl = [p for p in plans if t0 - 0.5 <= p["sim"] <= t1]
    lasts = {(round(p["x"][-1], 4), round(p["y"][-1], 4), round(float(p["yaw"][-1]), 4)) for p in pl}
    rv, cu = cusps_reverse(pl[0]) if pl and len(pl[0]["x"]) > 2 else (float("nan"), -1)
    # terminal analysis: after first time the robot is within XY tol
    inside = np.where(dist <= XY_TOL)[0]; term = {}
    if len(inside):
        i0 = inside[0]; tp = ph  # phases that start after entering tolerance
        after = [p for p in ph if p[1] + t0 >= T[i0] - 0.06]
        term = {"t_enter_xy_tol_s": T[i0] - t0, "mppi_to_shim_handoffs_after_entry": sum(1 for a, b in zip(M[i0:-1], M[i0 + 1:]) if a and not b),
                "shim_to_mppi_handoffs_after_entry": sum(1 for a, b in zip(M[i0:-1], M[i0 + 1:]) if (not a) and b),
                "max_dist_after_entry_m": float(dist[i0:].max()), "phases_after_entry": [(p[0], round(p[1], 2), round(p[2], 2)) for p in after]}
    sh = [p for p in ph if p[0] == "SHIM"]
    gtf = (G(t1, gt_x), G(t1, gt_y), float(wrap(G(t1, gt_yaw) - gyaw)))
    # speeds at end (odom, fused measure of what the plant did)
    ko = (odom[:, 0] > t1 - 0.3) & (odom[:, 0] <= t1 + 0.05)
    rep.append(f"## {nm}: {gr['status']}  (goal {gx:.3f},{gy:.3f},yaw {gyaw:.3f}; sim {t1 - t0:.1f} s; error_code {gr.get('error_code')})\n"
               f"- GT final: dist {math.hypot(gtf[0] - gx, gtf[1] - gy):.3f} m, yaw err {math.degrees(gtf[2]):+.1f} deg; odom wz {odom[ko, 2].mean() if ko.any() else float('nan'):+.3f} vx {odom[ko, 1].mean() if ko.any() else float('nan'):+.3f} (last 0.3 s)\n"
               f"- plans in window: {len(pl)}; distinct last poses: {len(lasts)} {sorted(lasts)[:3]}; first plan reverse {rv:.2f} m, cusps {cu}\n"
               f"- cmd_vel_nav samples {len(T)}: MPPI {int(M.sum())}, SHIM {int((~M).sum())}; terminal: {json.dumps(term, default=float)}\n"
               f"- phases (src, t0, t1, d_yaw_gt[rad], dist_start, dist_end, max|wz|cmd, max|vx|cmd, n):\n" +
               "\n".join(f"    {p[0]:4s} {p[1]:6.2f}-{p[2]:6.2f} s  dyaw {p[3]:+.2f}  dist {p[4]:.2f}->{p[5]:.2f}  |wz|<={p[6]:.2f} |vx|<={p[7]:.2f} n={p[8]}" for p in ph) +
               f"\n- PRE-TERMINAL shim segments (start dist > xy_tol+0.06 m): {len([p for p in sh if p[4] > XY_TOL + 0.06])}, total |dyaw| {math.degrees(sum(abs(p[3]) for p in sh if p[4] > XY_TOL + 0.06)):.0f} deg; xy_tol used {XY_TOL}\n- shim segments: {len(sh)}; max |wz| in shim = {max([p[6] for p in sh], default=0):.2f} (config 0.8); max |dyaw| in a shim segment = {math.degrees(max([abs(p[3]) for p in sh], default=0)):.0f} deg\n")
    # what Nav2's goal checker saw: it compares the AMCL pose with the LAST POSE OF THE PLANNED PATH (controller_server end_pose_), not the requested goal
    ia = int(np.argmin(np.abs(amcl[:, 0] - t1))); ae = amcl[ia]; lp = [p for p in plans if p["sim"] <= t1]; lp = lp[-1] if lp else None
    if lp is not None:
        ex, ey_, eyw = float(lp["x"][-1]), float(lp["y"][-1]), float(lp["yaw"][-1])
        rep.append(f"- at success (AMCL estimate): to REQUESTED goal dist {math.hypot(ae[1] - gx, ae[2] - gy):.3f} m, yaw {math.degrees(wrap(ae[3] - gyaw)):+.1f} deg | "
                   f"to PLAN END POSE ({ex:.3f},{ey_:.3f},{math.degrees(eyw):.1f} deg) dist {math.hypot(ae[1] - ex, ae[2] - ey_):.3f} m, yaw {math.degrees(wrap(ae[3] - eyw)):+.1f} deg | est-GT yaw {math.degrees(wrap(ae[3] - G(t1, gt_yaw))):+.1f} deg\n")
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.2))
    kk = (gt_t >= t0) & (gt_t <= t1); ax[0].plot(gt_x[kk], gt_y[kk], "0.7", lw=1)
    for src, c in (("MPPI", "C0"), ("SHIM", "C3")):
        m_ = M if src == "MPPI" else ~M; ax[0].scatter(G(T[m_], gt_x), G(T[m_], gt_y), s=6, c=c, label=src)
    ax[0].add_patch(plt.Circle((gx, gy), XY_TOL, fill=False, ls="--")); ax[0].annotate("", (gx + .3 * math.cos(gyaw), gy + .3 * math.sin(gyaw)), (gx, gy), arrowprops=dict(arrowstyle="->"))
    ax[0].set_aspect("equal"); ax[0].legend(); ax[0].set_title(f"{nm}: GT path by command source"); ax[0].grid(alpha=.3)
    for src, c in (("MPPI", "C0"), ("SHIM", "C3")):
        m_ = M if src == "MPPI" else ~M; ax[1].scatter(T[m_] - t0, WZ[m_], s=6, c=c)
    ks = (smo[:, 0] >= t0) & (smo[:, 0] <= t1); ko_ = (odom[:, 0] >= t0) & (odom[:, 0] <= t1)
    ax[1].plot(smo[ks, 0] - t0, smo[ks, 2], "C2", lw=.8, label="cmd_vel_smoothed wz"); ax[1].plot(odom[ko_, 0] - t0, odom[ko_, 2], "k", lw=.8, label="odom wz")
    ax[1].axhline(.8, c="r", ls=":", lw=.6); ax[1].axhline(-.8, c="r", ls=":", lw=.6); ax[1].set_title("angular velocity [rad/s]"); ax[1].legend(fontsize=7); ax[1].grid(alpha=.3)
    ax[2].plot(T - t0, dist, "C1", label="dist to goal [m]"); ax[2].plot(T - t0, np.degrees(np.abs(yerr)) / 100, "C4", label="|yaw err| [deg/100]")
    ax[2].axhline(XY_TOL, c="C1", ls=":"); ax[2].axhline(math.degrees(YAW_TOL) / 100, c="C4", ls=":"); ax[2].set_xlabel("s since goal accepted"); ax[2].legend(fontsize=7); ax[2].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(out / f"{nm}.png", dpi=110); plt.close(fig)
(out / "summary.txt").write_text("\n".join(rep)); print("\n".join(rep))
