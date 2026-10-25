#!/usr/bin/env python3
"""Send one NavigateToPose goal and watch it with nav_watchdog.py (pathological rotation / no progress).

Goals (map frame, from src/amr_missions/config/stations.yaml):
  station_b   (8.716, 2.705, yaw 0)
  staging_a   (0.099, -0.175, yaw 0)   = station A (0.799, -0.175) - 0.70 m dock staging offset
The watchdog rules and thresholds live in nav_watchdog.py / nav_watchdog.yaml (override with --config or
--set key=value); a trigger cancels the goal and is recorded in <out>/<label>_result.json.
Nav2 parameters are never touched.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from nav_watchdog import Config, Watchdog

GOALS = {"station_b": (8.716, 2.705, 0.0), "staging_a": (0.099, -0.175, 0.0)}
STATUS = {v: k for k, v in vars(GoalStatus).items() if k.startswith("STATUS_")}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Goal(Node):
    def __init__(self):
        super().__init__("nav_goal_client", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd = None
        self.create_subscription(TwistStamped, "/cmd_vel", lambda m: setattr(self, "cmd", m), 10)
        self.fb = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("goal", choices=GOALS)
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--config", default=str(Path(__file__).with_name("nav_watchdog.yaml")))
    ap.add_argument("--timeout", type=float, default=None, help="shorthand for --set timeout=SECONDS")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="override watchdog config values")
    ap.add_argument("--no-watchdog", action="store_true")
    a = ap.parse_args()
    gx, gy, gyaw = GOALS[a.goal]
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cfg = Config.load(a.config, timeout=a.timeout, **{k: type(getattr(Config(), k))(v) for k, v in (x.split("=", 1) for x in a.set)})
    wd = Watchdog(cfg, (gx, gy))

    rclpy.init()
    n = Goal()
    if not n.client.wait_for_server(timeout_sec=60):
        print("navigate_to_pose server not available"); return 2
    for _ in range(30):                       # let /clock and /cmd_vel discovery settle
        rclpy.spin_once(n, timeout_sec=0.1)

    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)

    res = {"goal": a.goal, "goal_xy_yaw": [gx, gy, gyaw], "args": vars(a), "abort_reason": None}
    t_wall0 = time.time()
    res["sim_time_goal_sent"] = n.now(); res["wall_goal_sent"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    fut = n.client.send_goal_async(g, feedback_callback=lambda m: setattr(n, "fb", m.feedback))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
    gh = fut.result()
    if gh is None or not gh.accepted:
        res["status"] = "REJECTED"; (out / f"{a.label}_result.json").write_text(json.dumps(res, indent=2)); return 1
    rfut = gh.get_result_async()
    t0 = n.now()
    last_t, cancelled = None, False
    min_dist, trace = 1e9, []
    while rclpy.ok() and not rfut.done():
        rclpy.spin_once(n, timeout_sec=0.1)
        t = n.now()
        if n.fb is None:
            continue
        p = n.fb.current_pose.pose
        d = math.hypot(p.position.x - gx, p.position.y - gy)
        yw = yaw_of(p.orientation)
        min_dist = min(min_dist, d)
        wz = n.cmd.twist.angular.z if n.cmd else 0.0
        vx = n.cmd.twist.linear.x if n.cmd else 0.0
        if last_t is None or t - last_t >= 1.0:       # 1 s sim-time trace
            trace.append([round(t - t0, 2), p.position.x, p.position.y, yw, d, wrap(yw - gyaw), vx,
                          wz if n.cmd else float("nan"), n.fb.number_of_recoveries])
            last_t = t
        if a.no_watchdog or cancelled:
            continue
        trig = wd.update(t, p.position.x, p.position.y, yw, wz, vx, n.fb.distance_remaining,
                         n.fb.number_of_recoveries)
        if trig:
            reason = f"[{trig.rule}] {trig.reason}"
            print("ABORT:", reason, flush=True)
            res["abort_reason"], cancelled = reason, True
            res["sim_time_abort"] = t
            gh.cancel_goal_async()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.1)
    r = rfut.result() if rfut.done() else None
    res["status"] = STATUS.get(r.status, r.status) if r else "NO_RESULT"
    res["error_code"] = int(getattr(r.result, "error_code", -1)) if r else None
    res["error_msg"] = getattr(r.result, "error_msg", "") if r else ""
    res["sim_duration_s"] = n.now() - t0
    res["wall_duration_s"] = time.time() - t_wall0
    res["min_dist_to_goal_xy"] = min_dist
    res["last_feedback"] = None if n.fb is None else {
        "x": n.fb.current_pose.pose.position.x, "y": n.fb.current_pose.pose.position.y,
        "yaw": yaw_of(n.fb.current_pose.pose.orientation),
        "distance_remaining": n.fb.distance_remaining, "recoveries": n.fb.number_of_recoveries}
    res["cmd_after_end"] = None if n.cmd is None else [n.cmd.twist.linear.x, n.cmd.twist.angular.z]
    res["watchdog"] = wd.summary()
    res["trace_columns"] = ["t_since_goal", "x", "y", "yaw", "dist_xy", "yaw_err", "cmd_vx", "cmd_wz", "recoveries"]
    res["trace"] = trace
    (out / f"{a.label}_result.json").write_text(json.dumps(res, indent=2))
    print(f"{a.label}: {res['status']} in {res['sim_duration_s']:.1f} sim-s; abort={res['abort_reason']}")
    n.destroy_node(); rclpy.try_shutdown()
    return 0 if res["status"] == "STATUS_SUCCEEDED" else 1


if __name__ == "__main__":
    sys.exit(main())
