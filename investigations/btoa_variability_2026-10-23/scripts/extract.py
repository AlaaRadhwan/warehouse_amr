#!/usr/bin/env python3
"""Read a run's rosbag2 (mcap) into one compact pickle for analysis.  Read-only on the bag.
usage: extract.py RUN_DIR [--bag DIR] [--out FILE]
Bag log_time is WALL time (recorded without --use-sim-time); header stamps are SIM time.
Unstamped messages (action feedback/status) are placed on the sim timeline by interpolating
/diff_drive_controller/odom (wall -> sim)."""
import argparse, gzip, math, pickle
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--bag"); ap.add_argument("--out")
a = ap.parse_args(); run = Path(a.run)
bag = a.bag or str(run / "bag"); out = a.out or str(run / "extracted.pkl.gz")
yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
D = {k: [] for k in ("cmd_nav", "cmd_smoothed", "cmd", "cmd_out", "odom_raw", "odom_fused", "amcl", "map_odom", "imu", "joints", "feedback",
                     "nav_status", "follow_status", "plan_status", "bt", "rosout", "coll", "costmap", "scan_n")}
D["paths"] = {"plan": [], "plan_smoothed": [], "unsmoothed": [], "transformed": [], "optimal": []}
PATHS = {"/plan": "plan", "/plan_smoothed": "plan_smoothed", "/unsmoothed_plan": "unsmoothed", "/transformed_global_plan": "transformed", "/optimal_trajectory": "optimal"}
CMDS = {"/cmd_vel_nav": "cmd_nav", "/cmd_vel_smoothed": "cmd_smoothed", "/cmd_vel": "cmd", "/diff_drive_controller/cmd_vel_out": "cmd_out"}
KEEP = ("progress", "recover", "abort", "goal", "cusp", "invert", "fail", "missed", "path", "timeout", "extrapolat", "collision", "stop", "cancel", "Optimizer")
while r.has_next():
    topic, raw, tns = r.read_next(); wall = tns * 1e-9; m = deserialize_message(raw, types[topic])
    if topic in CMDS:
        D[CMDS[topic]].append((st(m.header), wall, m.twist.linear.x, m.twist.angular.z))
    elif topic in PATHS:
        p = m.poses
        D["paths"][PATHS[topic]].append({"sim": st(m.header), "wall": wall, "frame": m.header.frame_id, "x": np.array([q.pose.position.x for q in p]), "y": np.array([q.pose.position.y for q in p]),
                                         "yaw": np.array([yawq(q.pose.orientation) for q in p])})
    elif topic in ("/diff_drive_controller/odom", "/odom"):
        k = "odom_raw" if topic.startswith("/diff") else "odom_fused"; q = m.pose.pose
        D[k].append((st(m.header), wall, q.position.x, q.position.y, yawq(q.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif topic == "/amcl_pose":
        c = m.pose.covariance; q = m.pose.pose
        D["amcl"].append((st(m.header), wall, q.position.x, q.position.y, yawq(q.orientation), c[0], c[7], c[35]))
    elif topic == "/tf":
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                D["map_odom"].append((st(t.header), wall, t.transform.translation.x, t.transform.translation.y, yawq(t.transform.rotation)))
    elif topic == "/imu/data_raw":
        D["imu"].append((st(m.header), wall, m.angular_velocity.z))
    elif topic == "/joint_states":
        n = list(m.name); D["joints"].append((st(m.header), wall, *[m.velocity[n.index(j)] if j in n and len(m.velocity) > n.index(j) else float("nan") for j in ("left_wheel_joint", "right_wheel_joint")]))
    elif topic == "/navigate_to_pose/_action/feedback":
        f = m.feedback; q = f.current_pose.pose
        D["feedback"].append((wall, q.position.x, q.position.y, yawq(q.orientation), f.distance_remaining, f.number_of_recoveries, f.navigation_time.sec + f.navigation_time.nanosec * 1e-9))
    elif topic in ("/navigate_to_pose/_action/status", "/follow_path/_action/status", "/compute_path_to_pose/_action/status"):
        k = {"/navigate_to_pose/_action/status": "nav_status", "/follow_path/_action/status": "follow_status"}.get(topic, "plan_status")
        D[k].append((wall, [(s.status, bytes(s.goal_info.goal_id.uuid).hex()[:8]) for s in m.status_list]))
    elif topic == "/behavior_tree_log":
        for e in m.event_log:
            D["bt"].append((e.timestamp.sec + e.timestamp.nanosec * 1e-9, wall, e.node_name, e.previous_status, e.current_status))
    elif topic == "/rosout":
        if m.level >= 30 or any(s in m.msg for s in KEEP):
            D["rosout"].append((st(m), wall, m.level, m.name, m.msg))
    elif topic == "/collision_monitor_state":
        D["coll"].append((wall, int(m.action_type), str(m.polygon_name)))
    elif topic == "/local_costmap/costmap":
        D["costmap"].append({"sim": st(m.header), "wall": wall, "frame": m.header.frame_id, "res": m.info.resolution, "w": m.info.width, "h": m.info.height,
                             "ox": m.info.origin.position.x, "oy": m.info.origin.position.y, "data": np.array(m.data, dtype=np.int8)})
    elif topic == "/scan":
        D["scan_n"].append((st(m.header), wall))
for k, v in D.items():
    if isinstance(v, list) and v and not isinstance(v[0], dict): D[k] = np.array(v, dtype=object if k in ("nav_status", "follow_status", "plan_status", "bt", "rosout", "coll") else float)
# wall -> sim map from raw odometry
o = D["odom_raw"]; D["wall2sim"] = np.array([[x[1], x[0]] for x in o], dtype=float) if len(o) else np.zeros((0, 2))
pickle.dump(D, gzip.open(out, "wb")); print("wrote", out, {k: (len(v) if hasattr(v, "__len__") else v) for k, v in D.items() if k != "paths"}, {k: len(v) for k, v in D["paths"].items()})
