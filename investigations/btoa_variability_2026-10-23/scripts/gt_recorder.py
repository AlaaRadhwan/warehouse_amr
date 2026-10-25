#!/usr/bin/env python3
"""Gazebo ground-truth pose -> CSV, flushed incrementally so a killed run still leaves data.

Same source as calibration/nav_validation/record_gt.py (gz-transport /world/<world>/pose/info) but
(a) rows are written as they arrive, (b) no ROS node / no /clock subscription (lighter), and
(c) each row carries wall time as well as the simulator stamp.
usage: gt_recorder.py OUT.csv [--world warehouse_world] [--model warehouse_amr]
"""
import argparse, csv, signal, time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

ap = argparse.ArgumentParser()
ap.add_argument("out"); ap.add_argument("--world", default="warehouse_world"); ap.add_argument("--model", default="warehouse_amr")
a = ap.parse_args()
f = open(a.out, "w", newline=""); w = csv.writer(f)
w.writerow(["wall", "sim", "x", "y", "z", "qx", "qy", "qz", "qw"]); f.flush()
n = [0]


def cb(m):
    for p in m.pose:
        if p.name == a.model:
            w.writerow((f"{time.time():.4f}", f"{m.header.stamp.sec + m.header.stamp.nsec * 1e-9:.4f}", p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
            n[0] += 1
            if n[0] % 50 == 0:
                f.flush()


gz = GzNode(); gz.subscribe(Pose_V, f"/world/{a.world}/pose/info", cb)
stop = []
signal.signal(signal.SIGINT, lambda *_: stop.append(1)); signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
while not stop:
    time.sleep(0.1)
f.flush(); f.close(); print(f"wrote {n[0]} rows to {a.out}")
