#!/usr/bin/env python3
"""Record Gazebo ground-truth pose of the robot model to CSV (sim time) until SIGINT/SIGTERM.

Same source as calibration/yaw_rate_sysid (gz-transport /world/<world>/pose/info).
Usage: record_gt.py OUT.csv [--world warehouse_world] [--model warehouse_amr]
"""
import argparse
import csv
import signal

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.parameter import Parameter

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--world", default="warehouse_world")
ap.add_argument("--model", default="warehouse_amr")
a = ap.parse_args()

rclpy.init()
n = Node("gt_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
rows = []


def cb(m):
    t = n.get_clock().now().nanoseconds * 1e-9
    s = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
    for p in m.pose:
        if p.name == a.model:
            rows.append((t, s, p.position.x, p.position.y, p.position.z,
                         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))


gz = GzNode()
gz.subscribe(Pose_V, f"/world/{a.world}/pose/info", cb)
stop = []
signal.signal(signal.SIGINT, lambda *_: stop.append(1))
signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
while not stop:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(a.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_recv", "t_stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
    w.writerows(rows)
print(f"wrote {len(rows)} rows to {a.out}")
