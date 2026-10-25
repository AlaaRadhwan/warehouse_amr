#!/usr/bin/env python3
"""Send the B -> A staging NavigateToPose goal and manage its lifetime for the variability investigation.

* Goal identical to calibration/nav_validation/nav_goal.py (GOALS['staging_a']); the existing, unmodified
  nav_watchdog.Watchdog supplies the online failure detection.
* Ends the run on whichever comes first: Nav2 result | online watchdog trigger + POST_TRIGGER sim-seconds of extra
  evidence | SIM-time limit | wall-clock safety limit.  Then cancels the goal (clean Nav2 state) and records why.
* Everything is written incrementally (client_trace.csv, client_events.jsonl) and client_result.json is written
  even on SIGTERM/SIGINT/exception, so a timed-out or killed run still leaves its record.
* Observation only: no Nav2 parameter is read or written.
usage: btoa_client.py OUT_DIR [--sim-limit S] [--wall-limit S] [--post-trigger S] [--config YAML]
"""
import argparse, csv, json, math, signal, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "calibration/nav_validation"))

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_goal import GOALS, STATUS, wrap, yaw_of          # existing, unmodified helpers
from nav_watchdog import Config, Watchdog
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

ap = argparse.ArgumentParser()
ap.add_argument("out"); ap.add_argument("--sim-limit", type=float, default=120.0)
ap.add_argument("--wall-limit", type=float, default=330.0); ap.add_argument("--post-trigger", type=float, default=10.0)
ap.add_argument("--config", default=str(Path(__file__).resolve().parents[1] / "config/watchdog_experiment.yaml"))
a = ap.parse_args()
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
gx, gy, gyaw = GOALS["staging_a"]
res = {"goal": "staging_a", "goal_xy_yaw": [gx, gy, gyaw], "sim_limit_s": a.sim_limit, "wall_limit_s": a.wall_limit,
       "post_trigger_s": a.post_trigger, "end_reason": None, "status": None, "watchdog_trigger": None}
ev = open(out / "client_events.jsonl", "a")
tr = open(out / "client_trace.csv", "w", newline=""); trw = csv.writer(tr)
trw.writerow(["t_goal_sim", "wall", "sim", "x", "y", "yaw", "dist_xy", "dist_remaining", "cmd_vx", "cmd_wz", "recoveries"]); tr.flush()


def event(kind, **kw):
    ev.write(json.dumps({"kind": kind, "wall": time.time(), **kw}) + "\n"); ev.flush()


def finish(code=0):
    res["wall_finished"] = time.time()
    (out / "client_result.json").write_text(json.dumps(res, indent=2, default=str)); sys.exit(code)


signal.signal(signal.SIGTERM, lambda *_: (res.__setitem__("end_reason", res["end_reason"] or "client_sigterm"), finish(2)))

rclpy.init()
n = Node("btoa_client", parameter_overrides=[Parameter("use_sim_time", value=True)])
client = ActionClient(n, NavigateToPose, "navigate_to_pose")
st = {"cmd": None, "fb": None}
n.create_subscription(TwistStamped, "/cmd_vel", lambda m: st.__setitem__("cmd", m), 10)
now = lambda: n.get_clock().now().nanoseconds * 1e-9
try:
    if not client.wait_for_server(timeout_sec=60):
        res["end_reason"] = "action_server_unavailable"; finish(3)
    for _ in range(30): rclpy.spin_once(n, timeout_sec=0.1)
    wd = Watchdog(Config.load(a.config), (gx, gy))
    g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = "map"; g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    res["sim_goal_sent"], res["wall_goal_sent"] = now(), time.time(); event("goal_sent", sim=now())
    fut = client.send_goal_async(g, feedback_callback=lambda m: st.__setitem__("fb", m.feedback))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
    gh = fut.result()
    if gh is None or not gh.accepted:
        res["end_reason"], res["status"] = "goal_rejected", "REJECTED"; finish(1)
    event("goal_accepted", sim=now()); rfut = gh.get_result_async()
    t0, wall0, last_log, cancel_at, trig_t, cancelled = now(), time.time(), None, None, None, False
    while rclpy.ok() and not rfut.done():
        rclpy.spin_once(n, timeout_sec=0.1)
        t, fb = now(), st["fb"]
        if fb is not None:
            p = fb.current_pose.pose; yw = yaw_of(p.orientation); d = math.hypot(p.position.x - gx, p.position.y - gy)
            cmd = st["cmd"]; wz = cmd.twist.angular.z if cmd else 0.0; vx = cmd.twist.linear.x if cmd else 0.0
            if last_log is None or t - last_log >= 1.0:
                trw.writerow([round(t - t0, 2), round(time.time(), 2), round(t, 2), p.position.x, p.position.y, yw, d,
                              fb.distance_remaining, vx, wz if cmd else "", fb.number_of_recoveries]); tr.flush(); last_log = t
            if not cancelled and trig_t is None:
                trig = wd.update(t, p.position.x, p.position.y, yw, wz, vx, fb.distance_remaining, fb.number_of_recoveries)
                if trig:
                    trig_t = t; res["watchdog_trigger"] = {"rule": trig.rule, "reason": trig.reason, "t_goal_sim": t - t0}
                    event("watchdog_trigger", rule=trig.rule, reason=trig.reason, t_goal_sim=t - t0)
        why = None
        if trig_t is not None and t - trig_t >= a.post_trigger: why = f"watchdog:{res['watchdog_trigger']['rule']}"
        elif t - t0 >= a.sim_limit: why = "sim_limit"
        elif time.time() - wall0 >= a.wall_limit: why = "wall_limit"
        if why and not cancelled:
            res["end_reason"], cancelled, cancel_at = why, True, time.time()
            event("cancel_sent", reason=why, t_goal_sim=t - t0); gh.cancel_goal_async()
        if cancelled and time.time() - cancel_at > 10: break          # cancel not acknowledged: stop waiting
    for _ in range(20): rclpy.spin_once(n, timeout_sec=0.1)
    r = rfut.result() if rfut.done() else None
    res["status"] = STATUS.get(r.status, r.status) if r else "NO_RESULT"
    res["error_code"] = int(getattr(r.result, "error_code", -1)) if r else None
    res["error_msg"] = getattr(r.result, "error_msg", "") if r else ""
    if res["end_reason"] is None: res["end_reason"] = "nav2_result"
    res["sim_duration_s"], res["wall_duration_s"] = now() - t0, time.time() - wall0
    fb = st["fb"]
    res["last_feedback"] = None if fb is None else {"x": fb.current_pose.pose.position.x, "y": fb.current_pose.pose.position.y,
        "yaw": yaw_of(fb.current_pose.pose.orientation), "distance_remaining": fb.distance_remaining, "recoveries": fb.number_of_recoveries}
    event("finished", status=res["status"], reason=res["end_reason"]); finish(0)
except SystemExit: raise
except BaseException as e:
    res["end_reason"] = res["end_reason"] or f"client_exception:{type(e).__name__}"; res["exception"] = repr(e); finish(4)
