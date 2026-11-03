#!/usr/bin/env python3
"""Send a fixed sequence of NavigateToPose goals (Rotation Shim functional checks). Observation only.
Each goal: sim-time limit + wall-clock limit, cancel on timeout, result written incrementally to client_result.json.
Goal poses are in the map frame; the run starts at Station B.
usage: shim_client.py RUN_DIR [--goalset full]
"""
import argparse, json, math, sys, time
from pathlib import Path
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

H = math.pi / 2
NONBIN = 0.2318   # midpoint of the lattice heading gap 0 .. 0.4636 rad: 0.232 rad from BOTH neighbouring bins (yaw tolerance 0.20)
GOALSETS = {   # name, x, y, yaw, sim_limit_s
    "full": [
        ("B_staging_A",        0.099, -0.175, 0.0,   120),   # Test B: established B->A scenario (2 cusps, ~2.6 m reverse in baseline)
        ("C_reverse_1m",      -0.900, -0.175, 0.0,    60),   # Test C: 1 m straight behind, same heading
        ("A1_fwd_turn90",     -0.300, -0.175, H,      60),   # Test A: 0.6 m forward, +90 deg final heading
        ("A2_inplace_turn180", -0.300, -0.175, -H,    40),   # Test A: same XY, 180 deg heading change
    ],
}
GOALSETS["p2"] = GOALSETS["full"] + [("N1_nonbin_yaw", -0.300, -0.175, NONBIN, 45)]   # Task 4: requested yaw not on a lattice heading
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--goalset", default="full"); a = ap.parse_args()
out = Path(a.run); res = {"goals": []}
ev = open(out / "client_events.jsonl", "a")
def event(kind, **kw): ev.write(json.dumps({"kind": kind, "wall": time.time(), **kw}) + "\n"); ev.flush()
def save(): (out / "client_result.json").write_text(json.dumps(res, indent=2, default=str))
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "CANCELED"}

rclpy.init(); n = Node("shim_client", parameter_overrides=[Parameter("use_sim_time", value=True)])
cl = ActionClient(n, NavigateToPose, "navigate_to_pose"); fbs = {"fb": None}
now = lambda: n.get_clock().now().nanoseconds * 1e-9
try:
    if not cl.wait_for_server(timeout_sec=60): res["error"] = "action_server_unavailable"; save(); sys.exit(3)
    for _ in range(30): rclpy.spin_once(n, timeout_sec=0.1)
    for name, gx, gy, gyaw, lim in GOALSETS[a.goalset]:
        g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = "map"; g.pose.header.stamp = n.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        r = {"name": name, "goal": [gx, gy, gyaw], "sim_limit_s": lim, "sim_sent": now(), "wall_sent": time.time()}
        fbs["fb"] = None; event("goal_sent", name=name, sim=now(), goal=[gx, gy, gyaw])
        fut = cl.send_goal_async(g, feedback_callback=lambda m: fbs.__setitem__("fb", m.feedback))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20); gh = fut.result()
        if gh is None or not gh.accepted:
            r["status"] = "REJECTED"; res["goals"].append(r); save(); event("rejected", name=name); continue
        r["sim_accepted"] = now(); event("goal_accepted", name=name, sim=now()); rf = gh.get_result_async()
        t0, w0, cancelled = now(), time.time(), False
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if not cancelled and (now() - t0 >= lim or time.time() - w0 >= lim * 3):
                cancelled = True; r["cancel_reason"] = "sim_limit" if now() - t0 >= lim else "wall_limit"
                event("cancel_sent", name=name, sim=now()); gh.cancel_goal_async()
            if cancelled and time.time() - w0 > lim * 3 + 15: break
        for _ in range(20): rclpy.spin_once(n, timeout_sec=0.1)
        rr = rf.result() if rf.done() else None
        r["status"] = STATUS.get(rr.status, rr.status) if rr else "NO_RESULT"
        r["error_code"] = int(getattr(rr.result, "error_code", -1)) if rr else None
        r["error_msg"] = getattr(rr.result, "error_msg", "") if rr else ""
        r["sim_finished"], r["sim_duration_s"], r["wall_duration_s"] = now(), now() - t0, time.time() - w0
        fb = fbs["fb"]
        if fb is not None:
            p = fb.current_pose.pose; r["last_feedback"] = {"x": p.position.x, "y": p.position.y,
                "yaw": 2 * math.atan2(p.orientation.z, p.orientation.w), "distance_remaining": fb.distance_remaining, "recoveries": fb.number_of_recoveries}
        res["goals"].append(r); save(); event("goal_finished", name=name, status=r["status"], sim=now())
        t_settle = now()
        while now() - t_settle < 3.0: rclpy.spin_once(n, timeout_sec=0.1)        # let the robot stop/settle between goals
except BaseException as e:
    res["exception"] = repr(e); save(); raise
save()
