#!/usr/bin/env python3
"""Replay the watchdog offline over recorded B->A bags (cmd_vel + action feedback).

Usage: replay_watchdog.py RUN_DIR [RUN_DIR ...] [--config nav_watchdog.yaml] [--goal 0.099 -0.175]
Prints per run: first trigger (sim time since first feedback, rule, reason), peak gated metrics.
"""
import argparse
import math
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from nav_watchdog import Config, Watchdog


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def replay(run, cfg, goal):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(Path(run) / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    fb_t, cmd_t = "/navigate_to_pose/_action/feedback", "/cmd_vel"
    cls = {k: get_message(types[k]) for k in (fb_t, cmd_t)}
    wd, cmd, t_first, n = Watchdog(cfg, goal), (0.0, 0.0), None, 0
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == cmd_t:
            m = deserialize_message(data, cls[topic]); cmd = (m.twist.linear.x, m.twist.angular.z)
        elif topic == fb_t:
            f = deserialize_message(data, cls[topic]).feedback
            t = t * 1e-9; t_first = t if t_first is None else t_first
            p = f.current_pose.pose; n += 1
            trig = wd.update(t, p.position.x, p.position.y, yaw_q(p.orientation), cmd[1], cmd[0],
                             f.distance_remaining, f.number_of_recoveries)
            if trig:
                return dict(run=Path(run).name, trigger=(trig.t - t_first, trig.rule, trig.reason),
                            peak=wd.peak, ctx=wd.ctx, duration=t - t_first, n=n)
    return dict(run=Path(run).name, trigger=None, peak=wd.peak, ctx=wd.ctx, duration=(t * 1e-9 - t_first) if t_first else 0, n=n)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--config", default=str(Path(__file__).with_name("nav_watchdog.yaml")))
    ap.add_argument("--goal", nargs=2, type=float, default=[0.099, -0.175])
    a = ap.parse_args()
    cfg = Config.load(a.config)
    for run in a.runs:
        res = replay(run, cfg, a.goal)
        print(f"{res['run']}: recorded {res['duration']:.1f}s, {res['n']} feedback msgs")
        print("   trigger:", "none" if not res["trigger"] else
              f"t={res['trigger'][0]:.1f}s rule={res['trigger'][1]} :: {res['trigger'][2]}")
        print("   context peaks:", {k: round(v, 2) for k, v in res["ctx"].items()})
        print("   gated margins (>0 = would trip; -9 = precondition never met):",
              {k: round(v, 2) for k, v in res["peak"].items()})


if __name__ == "__main__":
    sys.exit(main())
