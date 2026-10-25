#!/usr/bin/env python3
"""Offline analysis of one run_b_to_a.sh run directory (rosbag + ground truth + client result).

Outputs next to the run: b_to_a_report.md, b_to_a_metrics.json, b_to_a_overview.png,
b_to_a_terminal.png.  Read-only with respect to the bag and every configuration.

Goal / tolerances (from the installed nav2_params.yaml, StoppedGoalChecker):
  xy 0.25 m, yaw 0.20 rad, trans_stopped 0.05 m/s, rot_stopped 0.10 rad/s, stateful=false.
Ground truth is Gazebo's model pose; map == world frame is verified against the Nav2 feedback pose.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

GOAL = (0.099, -0.175, 0.0)
TOL = dict(xy=0.25, yaw=0.20, v=0.05, w=0.10)
NEAR = 1.0


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def read_bag(path):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    msgs = {k: [] for k in types}
    cls = {k: get_message(v) for k, v in types.items()}
    skip = {"/trajectories", "/clock", "/tf", "/tf_static", "/behavior_tree_log",
            "/joint_states", "/navigate_to_pose/_action/feedback"}   # bulky / not needed here
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in skip:
            continue
        msgs[topic].append((t * 1e-9, deserialize_message(data, cls[topic])))
    return msgs


def twist(m):
    return m.twist if hasattr(m, "twist") else m


def series_twist(msgs, topic):
    a = np.array([[t, twist(m).linear.x, twist(m).angular.z] for t, m in msgs.get(topic, [])])
    return a if len(a) else np.zeros((0, 3))


def series_odom(msgs, topic):
    a = np.array([[t, m.twist.twist.linear.x, m.twist.twist.angular.z,
                   yaw_q(m.pose.pose.orientation), m.pose.pose.position.x, m.pose.pose.position.y]
                  for t, m in msgs.get(topic, [])])
    return a if len(a) else np.zeros((0, 6))


def path_stats(m):
    p = m.poses
    if not p:
        return dict(n=0, length=0.0, cusps=0, rot_only=0, end=None, end_yaw=None)
    xy = np.array([[q.pose.position.x, q.pose.position.y] for q in p])
    yw = np.array([yaw_q(q.pose.orientation) for q in p])
    seg = np.diff(xy, axis=0)
    d = np.hypot(seg[:, 0], seg[:, 1])
    mv = d > 1e-3
    head = np.arctan2(seg[:, 1], seg[:, 0])
    fwd = np.cos(wrap(head - yw[:-1])) >= 0
    dirs = fwd[mv]
    cusps = int(np.sum(dirs[1:] != dirs[:-1])) if len(dirs) > 1 else 0
    rot_only = int(np.sum((~mv) & (np.abs(wrap(np.diff(yw))) > 1e-3)))
    return dict(n=len(p), length=float(d.sum()), cusps=cusps, rot_only=rot_only,
                reverse_frac=float(np.mean(~dirs)) if len(dirs) else 0.0,
                end=[float(xy[-1, 0]), float(xy[-1, 1])], end_yaw=float(yw[-1]),
                xy=xy.tolist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    a = ap.parse_args()
    run = Path(a.run)
    res = json.loads((run / "b_to_a_result.json").read_text())
    msgs = read_bag(run / "bag")
    gt = np.array([[float(x) for x in r] for r in list(csv.reader(open(run / "gt.csv")))[1:]])
    gt_t = gt[:, 1] if (gt[:, 1] > 0).all() else gt[:, 0]
    gt_yaw = np.unwrap([math.atan2(2 * (r[8] * r[7] + r[5] * r[6]), 1 - 2 * (r[6] ** 2 + r[7] ** 2)) for r in gt])
    gt_x, gt_y = gt[:, 2], gt[:, 3]

    # ---- time base: goal send (sim) from client; bag stamps are sim time (--use-sim-time)
    t_goal = res["sim_time_goal_sent"]
    t_end = t_goal + res["sim_duration_s"]
    sel = (gt_t >= t_goal - 1) & (gt_t <= t_end + 6)
    gt_t, gt_yaw, gt_x, gt_y = gt_t[sel], gt_yaw[sel], gt_x[sel], gt_y[sel]
    dist = np.hypot(gt_x - GOAL[0], gt_y - GOAL[1])
    yerr = wrap(gt_yaw - GOAL[2])
    # GT yaw rate (smoothed finite difference, 0.5 s)
    k = max(1, int(round(0.5 / np.median(np.diff(gt_t)))))
    gt_w = (gt_yaw[k:] - gt_yaw[:-k]) / (gt_t[k:] - gt_t[:-k])
    gt_wt = 0.5 * (gt_t[k:] + gt_t[:-k])
    gt_v = np.hypot(gt_x[k:] - gt_x[:-k], gt_y[k:] - gt_y[:-k]) / (gt_t[k:] - gt_t[:-k])

    # map == world check against Nav2 feedback pose
    fb = res["last_feedback"]
    j = np.argmin(np.abs(gt_t - t_end))
    d_loc = []
    for row in res["trace"]:
        tt_ = t_goal + row[0]
        d_loc.append(math.hypot(row[1] - np.interp(tt_, gt_t, gt_x), row[2] - np.interp(tt_, gt_t, gt_y)))
    loc_err = dict(mean_m=float(np.mean(d_loc)), max_m=float(np.max(d_loc)), final_m=float(d_loc[-1]))
    frame_check = dict(nav2_estimate_minus_ground_truth_xy=loc_err, feedback_xy=[fb["x"], fb["y"]], gt_xy=[float(gt_x[j]), float(gt_y[j])],
                       feedback_yaw=fb["yaw"], gt_yaw=float(wrap(gt_yaw[j])))

    cmd_nav = series_twist(msgs, "/cmd_vel_nav")
    cmd_sm = series_twist(msgs, "/cmd_vel_smoothed")
    cmd = series_twist(msgs, "/cmd_vel")
    cmd_out = series_twist(msgs, "/diff_drive_controller/cmd_vel_out")
    odom_raw = series_odom(msgs, "/diff_drive_controller/odom")
    odom_f = series_odom(msgs, "/odom")
    imu = np.array([[t, m.angular_velocity.z] for t, m in msgs.get("/imu/data_raw", [])])

    # ---- terminal metrics
    near = dist < NEAR
    t_near0 = float(gt_t[near][0]) if near.any() else None
    m = {"result": {k: res[k] for k in ("status", "abort_reason", "sim_duration_s", "wall_duration_s",
                                        "min_dist_to_goal_xy", "error_code", "error_msg")},
         "frame_check": frame_check}
    m["first_within_1m_s_after_goal"] = None if t_near0 is None else t_near0 - t_goal
    if near.any():
        tn = gt_t[near]
        wn = np.interp(tn, gt_wt, gt_w)
        dt = np.gradient(gt_t)[near]
        m["near_goal"] = dict(
            time_in_zone_s=float(gt_t[-1] - t_near0), max_abs_gt_wz=float(np.max(np.abs(wn))),
            time_abs_wz_gt_0p5_s=float(np.sum(dt[np.abs(wn) > 0.5])),
            total_rotation_deg=float(np.degrees(np.sum(np.abs(np.diff(gt_yaw[near]))))),
            signed_net_rotation_deg=float(np.degrees(gt_yaw[near][-1] - gt_yaw[near][0])))
    # forward/reverse reversals in the final command during the zone
    if len(cmd) and t_near0:
        c = cmd[(cmd[:, 0] >= t_near0)]
        s = np.sign(np.where(np.abs(c[:, 1]) < 0.02, 0, c[:, 1]))
        s = s[s != 0]
        m["near_goal_cmd_vx_sign_changes"] = int(np.sum(s[1:] != s[:-1])) if len(s) > 1 else 0
        m["near_goal_cmd_vx_min_max"] = [float(c[:, 1].min()), float(c[:, 1].max())]
        m["near_goal_cmd_wz_min_max"] = [float(c[:, 2].min()), float(c[:, 2].max())]
        m["near_goal_cmd_wz_mean_abs"] = float(np.mean(np.abs(c[:, 2])))
    # final state (GT) and checker conditions
    fin = slice(-5, None)
    m["final_gt"] = dict(x=float(gt_x[-1]), y=float(gt_y[-1]), dist_xy=float(dist[-1]),
                         yaw_err_deg=float(np.degrees(yerr[-1])), v=float(gt_v[-1]), wz=float(gt_w[-1]))
    ok = (dist[k:] <= TOL["xy"]) & (np.abs(yerr[k:]) <= TOL["yaw"]) & (gt_v <= TOL["v"]) & (np.abs(gt_w) <= TOL["w"])
    t_ok = float(gt_wt[ok][0]) if ok.any() else None
    m["checker_conditions_first_met_s_after_goal"] = None if t_ok is None else t_ok - t_goal
    m["checker_conditions_met_fraction_after_first"] = None if t_ok is None else float(np.mean(ok[gt_wt >= t_ok]))
    # velocity at the moment the action result arrived (GT)
    jr = np.argmin(np.abs(gt_wt - t_end))
    m["at_result"] = dict(gt_v=float(gt_v[jr]), gt_wz=float(gt_w[jr]), dist_xy=float(np.interp(t_end, gt_t, dist)),
                          yaw_err_deg=float(np.degrees(np.interp(t_end, gt_t, yerr))))
    # sensor yaw-rate agreement over the whole run while rotating fast
    def agree(arr, col):
        if not len(arr):
            return None
        tt = arr[:, 0]
        w = np.interp(tt, gt_wt, gt_w, left=np.nan, right=np.nan)
        mk = (np.abs(w) > 0.2) & np.isfinite(w)
        return None if mk.sum() < 10 else float(np.mean(arr[mk, col] / w[mk]))
    m["yaw_rate_ratio_to_gt_when_|wz|>0.2"] = dict(
        raw_wheel_odom=agree(odom_raw, 2), fused_odom=agree(odom_f, 2),
        imu=agree(imu, 1), cmd_vel=agree(cmd, 2), cmd_vel_nav=agree(cmd_nav, 2))
    # max commanded |wz| per stage
    m["cmd_wz_abs_max"] = {n: (float(np.max(np.abs(x[:, 2]))) if len(x) else None)
                           for n, x in (("cmd_vel_nav", cmd_nav), ("cmd_vel_smoothed", cmd_sm),
                                        ("cmd_vel", cmd), ("controller_out", cmd_out))}

    # ---- plans
    plans = {}
    for name in ("/unsmoothed_plan", "/plan", "/transformed_global_plan"):
        lst = [(t, path_stats(p)) for t, p in msgs.get(name, []) if t >= t_goal - 0.5]
        if not lst:
            plans[name] = None
            continue
        st = [s for _, s in lst]
        plans[name] = dict(
            count=len(st), first=dict(t=lst[0][0] - t_goal, **{k: v for k, v in st[0].items() if k != "xy"}),
            last=dict(t=lst[-1][0] - t_goal, **{k: v for k, v in st[-1].items() if k != "xy"}),
            max_length=float(max(s["length"] for s in st)), max_cusps=int(max(s["cusps"] for s in st)),
            msgs_with_cusp=int(sum(s["cusps"] > 0 for s in st)),
            msgs_with_reverse=int(sum(s.get("reverse_frac", 0) > 0.05 for s in st)),
            max_rot_only_poses=int(max(s["rot_only"] for s in st)),
            single_pose_msgs=int(sum(s["n"] <= 1 for s in st)))
        plans[name]["_series"] = [(t - t_goal, s) for t, s in lst]
    m["plans"] = {k: (None if v is None else {kk: vv for kk, vv in v.items() if kk != "_series"})
                  for k, v in plans.items()}
    # does the transformed plan's end stay at the true global goal?
    # /transformed_global_plan is in the odom frame: express the goal there using the
    # end-of-run offset between the fused /odom pose and the Nav2 (map) feedback pose.
    tp = plans.get("/transformed_global_plan")
    if tp and len(odom_f):
        j_ = np.argmin(np.abs(odom_f[:, 0] - t_end))
        off = np.array([odom_f[j_, 4] - fb["x"], odom_f[j_, 5] - fb["y"]])
        g_odom = np.array(GOAL[:2]) + off
        rows_ = []
        for t_, s_ in tp["_series"]:
            if not s_["end"]:
                continue
            jo = np.argmin(np.abs(odom_f[:, 0] - (t_ + t_goal)))
            rows_.append((t_, np.hypot(s_["end"][0] - g_odom[0], s_["end"][1] - g_odom[1]),
                          np.hypot(odom_f[jo, 4] - g_odom[0], odom_f[jo, 5] - g_odom[1]), s_["n"], s_["cusps"]))
        rw = np.array(rows_)
        zone = rw[:, 2] < NEAR
        m["transformed_plan"] = dict(
            frame="odom (goal mapped via end-of-run /odom minus feedback offset)",
            end_within_015m_frac_all=float(np.mean(rw[:, 1] < 0.15)),
            end_within_015m_frac_in_1m_zone=float(np.mean(rw[zone, 1] < 0.15)) if zone.any() else None,
            end_dist_to_goal_max_in_zone=float(rw[zone, 1].max()) if zone.any() else None,
            single_pose_frac_in_zone=float(np.mean(rw[zone, 3] <= 1)) if zone.any() else None,
            msgs_with_cusp_in_zone=int(np.sum(rw[zone, 4] > 0)) if zone.any() else None,
            msgs_in_zone=int(zone.sum()))
    # optimal trajectory predicted rotation
    ot = []
    for t, p in msgs.get("/optimal_trajectory", []):
        if t < t_goal or len(p.poses) < 2:
            continue
        ys = np.unwrap([yaw_q(q.pose.orientation) for q in p.poses])
        ot.append((t - t_goal, float(np.degrees(ys[-1] - ys[0]))))
    m["optimal_trajectory_pred_yaw_change_deg"] = None if not ot else dict(
        n=len(ot), mean_abs=float(np.mean(np.abs([o[1] for o in ot]))),
        last10_mean_abs=float(np.mean(np.abs([o[1] for o in ot[-10:]]))))
    m["feedback_trace_samples"] = len(res.get("trace", []))
    (run / "b_to_a_metrics.json").write_text(json.dumps(m, indent=2, default=float))

    # ---- plots
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tt = gt_t - t_goal
    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    ax[0, 0].plot(gt_x, gt_y, label="ground-truth path")
    first = plans["/unsmoothed_plan"]["_series"][0][1] if plans.get("/unsmoothed_plan") else None
    lastp = plans["/plan"]["_series"][-1][1] if plans.get("/plan") else None
    if first:
        xy = np.array(first["xy"]); ax[0, 0].plot(xy[:, 0], xy[:, 1], "--", label="first raw plan")
    if lastp:
        xy = np.array(lastp["xy"]); ax[0, 0].plot(xy[:, 0], xy[:, 1], ":", label="last smoothed plan")
    ax[0, 0].plot(*GOAL[:2], "r*", ms=14, label="goal"); ax[0, 0].set_aspect("equal", "datalim"); ax[0, 0].legend(fontsize=7)
    ax[0, 0].set_title("path (map/world frame)")
    ax[0, 1].plot(tt, dist, label="dist to goal [m]"); ax[0, 1].plot(tt, np.degrees(yerr) / 10, label="yaw error [deg/10]")
    ax[0, 1].axhline(TOL["xy"], color="gray", ls=":"); ax[0, 1].legend(); ax[0, 1].set_title("convergence (ground truth)")
    ax[1, 0].plot(gt_wt - t_goal, gt_w, "k", lw=2, label="GT")
    for arr, c, n_ in ((cmd, 2, "cmd_vel"), (odom_raw, 2, "raw odom"), (odom_f, 2, "fused odom")):
        if len(arr): ax[1, 0].plot(arr[:, 0] - t_goal, arr[:, c], lw=.8, label=n_)
    if len(imu): ax[1, 0].plot(imu[:, 0] - t_goal, imu[:, 1], lw=.8, label="IMU")
    ax[1, 0].set_title("yaw rate [rad/s]"); ax[1, 0].legend(fontsize=7)
    if len(cmd): ax[1, 1].plot(cmd[:, 0] - t_goal, cmd[:, 1], label="cmd_vel vx")
    ax[1, 1].plot(gt_wt - t_goal, gt_v, label="GT speed"); ax[1, 1].legend(); ax[1, 1].set_title("linear velocity [m/s]")
    for x_ in ax.flat[1:]: x_.set_xlabel("sim s since goal")
    fig.tight_layout(); fig.savefig(run / "b_to_a_overview.png", dpi=110)

    z = (gt_t >= (t_near0 or gt_t[0])) if near.any() else np.ones_like(gt_t, bool)
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].plot(tt[z], np.degrees(yerr[z])); ax[0].axhspan(-np.degrees(TOL["yaw"]), np.degrees(TOL["yaw"]), alpha=.2)
    ax[0].set_ylabel("GT yaw error [deg]")
    ax[1].plot(gt_wt[gt_wt >= gt_t[z][0]] - t_goal, gt_w[gt_wt >= gt_t[z][0]], "k", label="GT")
    for arr, c, n_ in ((cmd_nav, 2, "cmd_vel_nav (MPPI)"), (cmd, 2, "cmd_vel (final)")):
        if len(arr):
            s_ = arr[:, 0] >= gt_t[z][0]; ax[1].plot(arr[s_, 0] - t_goal, arr[s_, c], lw=.8, label=n_)
    ax[1].axhspan(-TOL["w"], TOL["w"], alpha=.2); ax[1].legend(fontsize=7); ax[1].set_ylabel("wz [rad/s]")
    ax[2].plot(tt[z], dist[z]); ax[2].axhline(TOL["xy"], color="gray", ls=":"); ax[2].set_ylabel("dist [m]")
    ax[2].set_xlabel("sim s since goal"); fig.tight_layout(); fig.savefig(run / "b_to_a_terminal.png", dpi=110)

    # ---- report
    L = [f"# B -> A navigation analysis: {run.name}", "",
         f"Result: **{res['status']}** after {res['sim_duration_s']:.1f} sim-s ({res['wall_duration_s']:.0f} wall-s); "
         f"watchdog abort: {res['abort_reason']}", "", "```json", json.dumps(m, indent=1, default=float), "```"]
    (run / "b_to_a_report.md").write_text("\n".join(L))
    print(json.dumps(m, indent=1, default=float))


if __name__ == "__main__":
    main()
