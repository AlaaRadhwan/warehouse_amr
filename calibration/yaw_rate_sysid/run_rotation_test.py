#!/usr/bin/env python3
"""Open-loop in-place rotation (--mode rotation, default) or straight-line
(--mode linear) system-identification run.

Commands +/- rotation steps through the diff_drive_controller input (/cmd_vel,
geometry_msgs/TwistStamped) while recording, on one sim-time base:

  cmd.csv        commanded yaw rate we publish (with phase labels)
  cmd_out.csv    controller-limited command (/diff_drive_controller/cmd_vel_out)
  gt.csv         Gazebo ground-truth model pose (gz-transport pose/info, native)
  odom_raw.csv   /diff_drive_controller/odom   (raw wheel odometry)
  joints.csv     /joint_states                 (wheel position + velocity)
  imu.csv        /imu/data_raw
  odom_fused.csv /odom                         (EKF output)
  metadata.json  schedule, controller params, file hashes, git rev

Nothing in the robot / Nav2 / EKF configuration is modified.  The script only
publishes /cmd_vel and subscribes to the Gazebo pose topic through the
gz-transport Python bindings (the ros_gz_bridge drops entity names there).

Abort:  Ctrl-C (or SIGTERM) -> zero command is published immediately, with no
ramp.  If this process is killed outright, the controller's cmd_vel_timeout
(0.5 s) stops the robot; `stop_robot.py` is a second, independent stop.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

REPO = Path(__file__).resolve().parents[2]
DEFAULT_STEPS = [0.4, -0.4, 0.7, -0.7, 1.0, -1.0]

CMD_TOPIC = "/cmd_vel"                                    # remapped from ~/cmd_vel
CMD_OUT_TOPIC = "/diff_drive_controller/cmd_vel_out"
RAW_ODOM_TOPIC = "/diff_drive_controller/odom"
FUSED_ODOM_TOPIC = "/odom"
IMU_TOPIC = "/imu/data_raw"
JOINT_TOPIC = "/joint_states"
CONTROLLER_NODE = "/diff_drive_controller"
CONTROLLER_PARAMS = [
    "open_loop", "position_feedback", "wheel_separation",
    "wheel_separation_multiplier", "wheel_radius",
    "left_wheel_radius_multiplier", "right_wheel_radius_multiplier",
    "cmd_vel_timeout", "publish_limited_velocity",
    "angular.z.max_acceleration", "angular.z.max_velocity",
]
NAV2_NODES = ["controller_server", "bt_navigator", "behavior_server",
              "velocity_smoother", "collision_monitor", "docking_server"]


def smootherstep(x):
    """Quintic 0->1 ramp with zero velocity and acceleration at both ends."""
    x = min(max(x, 0.0), 1.0)
    return x * x * x * (x * (6.0 * x - 15.0) + 10.0)


def quat_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def build_schedule(steps, baseline, hold, rest, max_accel):
    """Return list of (t0, t1, phase, step_idx, target_wz). t relative to start."""
    segs, t = [], 0.0
    segs.append((t, t + baseline, "baseline", -1, 0.0))
    t += baseline
    for i, w in enumerate(steps):
        # peak accel of the quintic ramp is 1.875 * w / T
        ramp = max(2.0, 1.875 * abs(w) / max_accel)
        segs.append((t, t + ramp, "ramp_up", i, w)); t += ramp
        segs.append((t, t + hold, "hold", i, w)); t += hold
        segs.append((t, t + ramp, "ramp_down", i, w)); t += ramp
        segs.append((t, t + rest, "rest", i, 0.0)); t += rest
    return segs


def cmd_at(segs, t):
    """(commanded wz, phase, step_idx, target) at time t since start, or None if done."""
    for t0, t1, phase, idx, w in segs:
        if t0 <= t < t1:
            x = (t - t0) / (t1 - t0)
            if phase == "ramp_up":
                return w * smootherstep(x), phase, idx, w
            if phase == "ramp_down":
                return w * (1.0 - smootherstep(x)), phase, idx, w
            if phase == "hold":
                return w, phase, idx, w
            return 0.0, phase, idx, w
    return None


def sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def sh(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except Exception as e:  # noqa: BLE001
        return 1, str(e)


def read_controller_params():
    out = {}
    for p in CONTROLLER_PARAMS:
        rc, txt = sh(["ros2", "param", "get", CONTROLLER_NODE, p])
        if rc == 0 and " is: " in txt:
            v = txt.split(" is: ", 1)[1].strip()
            if v in ("True", "False"):
                out[p] = (v == "True")
            else:
                try:
                    out[p] = float(v)
                except ValueError:
                    out[p] = v
        else:
            out[p] = None
    return out


class Runner(Node):
    def __init__(self, args):
        super().__init__("rotation_sysid", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args
        self.abort = False
        self.rows = {k: [] for k in
                     ("cmd", "cmd_out", "gt", "odom_raw", "joints", "imu", "odom_fused")}
        self.latest_gt = None
        self.gt_origin = None
        self.gt_names_seen = set()
        self.abort_reason = ""

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1000)
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(TwistStamped, CMD_OUT_TOPIC, self.on_cmd_out, qos)
        self.create_subscription(Odometry, RAW_ODOM_TOPIC, lambda m: self.on_odom(m, "odom_raw"), qos)
        self.create_subscription(Odometry, FUSED_ODOM_TOPIC, lambda m: self.on_odom(m, "odom_fused"), qos)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, qos)
        self.create_subscription(JointState, JOINT_TOPIC, self.on_joints, qos)
        self.gz_node = GzNode()
        if not self.gz_node.subscribe(Pose_V, args.gt_topic, self.on_gt):
            self.get_logger().error(f"cannot subscribe to gz topic {args.gt_topic}")

    # -- helpers ----------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    # -- callbacks --------------------------------------------------------
    def on_cmd_out(self, m):
        self.rows["cmd_out"].append((self.now(), self.stamp(m.header),
                                     m.twist.linear.x, m.twist.angular.z))

    def on_odom(self, m, key):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        self.rows[key].append((self.now(), self.stamp(m.header), p.x, p.y,
                               q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z))

    def on_imu(self, m):
        q, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
        self.rows["imu"].append((self.now(), self.stamp(m.header), q.x, q.y, q.z, q.w,
                                 w.x, w.y, w.z, a.x, a.y, a.z))

    def on_joints(self, m):
        d = {n: i for i, n in enumerate(m.name)}
        try:
            l, r = d["left_wheel_joint"], d["right_wheel_joint"]
        except KeyError:
            return
        self.rows["joints"].append((self.now(), self.stamp(m.header),
                                    m.position[l], m.position[r],
                                    m.velocity[l], m.velocity[r]))

    def on_gt(self, m):
        # gz-transport thread; list.append is atomic.
        t_recv = self.now()
        stamp = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        for p in m.pose:
            self.gt_names_seen.add(p.name)
            if p.name != self.args.model_name:
                continue
            t, q = p.position, p.orientation
            self.latest_gt = (t.x, t.y)
            if self.gt_origin is None:
                self.gt_origin = (t.x, t.y)
            self.rows["gt"].append((t_recv, stamp, t.x, t.y, t.z, q.x, q.y, q.z, q.w))

    # -- command ----------------------------------------------------------
    def send(self, wz):  # value goes to angular.z (rotation) or linear.x (linear)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        if self.args.mode == "linear":
            m.twist.linear.x = float(wz)
        else:
            m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def hard_stop(self, n=10):
        for _ in range(n):
            self.send(0.0)
            time.sleep(0.02)


def preflight(node, args):
    """Return (ok, info, problems)."""
    problems = []
    info = {}
    # sim clock must be running
    t0 = node.now()
    t_wall = time.time()
    while time.time() - t_wall < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.now() <= t0:
        problems.append("sim clock (/clock) is not advancing")

    # a foreign /cmd_vel publisher means Nav2 (or something) could fight us
    for pi in node.get_publishers_info_by_topic(CMD_TOPIC):
        if pi.node_name == node.get_name():
            continue
        full = f"{pi.node_namespace.rstrip('/')}/{pi.node_name}"
        rc, txt = sh(["ros2", "lifecycle", "get", full])
        state = txt.split()[0].lower() if rc == 0 and txt else "not-lifecycle"
        info.setdefault("foreign_cmd_vel_publishers", {})[full] = state
        if state != "inactive" and state != "unconfigured":
            problems.append(f"{CMD_TOPIC} also published by {full} (state: {state}). "
                            "Shut down / deactivate Nav2 first.")
    nodes = {f"{ns.rstrip('/')}/{n}" for n, ns in node.get_node_names_and_namespaces()}
    info["nav2_nodes_present"] = sorted(n for n in nodes
                                        if n.split("/")[-1] in NAV2_NODES)

    # controller must subscribe to /cmd_vel as TwistStamped
    subs = node.get_subscriptions_info_by_topic(CMD_TOPIC)
    if not any(s.node_name == "diff_drive_controller" for s in subs):
        problems.append("diff_drive_controller is not subscribed to /cmd_vel")

    # all data sources must be alive
    t_wall = time.time()
    while time.time() - t_wall < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    for key in ("odom_raw", "odom_fused", "imu", "joints", "gt"):
        if not node.rows[key]:
            msg = f"no data on source '{key}'"
            if key == "gt":
                msg += (f" (model '{args.model_name}' not in {args.gt_topic}; "
                        f"child frames seen: {sorted(node.gt_names_seen)[:15]})")
            problems.append(msg)
    return not problems, info, problems


def write_csvs(node, outdir):
    heads = {
        "cmd": (["t", "step", "phase", "target_v", "cmd_v"] if node.args.mode == "linear"
                else ["t", "step", "phase", "target_wz", "cmd_wz"]),
        "cmd_out": ["t_recv", "t_stamp", "vx", "wz"],
        "gt": ["t_recv", "t_stamp", "x", "y", "z", "qx", "qy", "qz", "qw"],
        "odom_raw": ["t_recv", "t_stamp", "x", "y", "qx", "qy", "qz", "qw", "vx", "vy", "wz"],
        "odom_fused": ["t_recv", "t_stamp", "x", "y", "qx", "qy", "qz", "qw", "vx", "vy", "wz"],
        "imu": ["t_recv", "t_stamp", "qx", "qy", "qz", "qw", "wx", "wy", "wz", "ax", "ay", "az"],
        "joints": ["t_recv", "t_stamp", "left_pos", "right_pos", "left_vel", "right_vel"],
    }
    for k, h in heads.items():
        with open(outdir / f"{k}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(h)
            w.writerows(node.rows[k])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("rotation", "linear"), default="rotation")
    ap.add_argument("--steps", type=float, nargs="+", default=None,
                    help="steps in rad/s (rotation) or m/s (linear), run in order")
    ap.add_argument("--hold", type=float, default=None, help="steady hold per step [s]; default 12 (rotation) / 8 (linear)")
    ap.add_argument("--baseline", type=float, default=3.0, help="initial zero-command record [s]")
    ap.add_argument("--rest", type=float, default=3.0, help="zero-command pause after each step [s]")
    ap.add_argument("--max-accel", type=float, default=None,
                    help="peak ramp accel; default 0.8 rad/s^2 (rotation, controller limit 1.5) or 0.5 m/s^2 (linear, limit 0.8)")
    ap.add_argument("--rate", type=float, default=50.0, help="/cmd_vel publish rate [Hz]")
    ap.add_argument("--world", default="warehouse_world")
    ap.add_argument("--model-name", default="warehouse_amr")
    ap.add_argument("--gt-topic", default=None,
                    help="default /world/<world>/pose/info")
    ap.add_argument("--max-drift", type=float, default=None,
                    help="abort if ground-truth xy is further than this from start [m]; default 0.5 (rotation) / 7.5 (linear)")
    ap.add_argument("--out", default=str(Path(__file__).parent / "runs"))
    ap.add_argument("--label", default="")
    ap.add_argument("--dry-run", action="store_true",
                    help="print schedule and exit (no ROS)")
    args = ap.parse_args()
    lin = args.mode == "linear"
    args.steps = args.steps or ([0.3, -0.3, 0.6, -0.6] if lin else DEFAULT_STEPS)
    args.hold = args.hold if args.hold is not None else (8.0 if lin else 12.0)
    args.max_accel = args.max_accel if args.max_accel is not None else (0.5 if lin else 0.8)
    args.max_drift = args.max_drift if args.max_drift is not None else (7.5 if lin else 0.5)
    args.gt_topic = args.gt_topic or f"/world/{args.world}/pose/info"

    segs = build_schedule(args.steps, args.baseline, args.hold, args.rest, args.max_accel)
    total = segs[-1][1]
    if args.dry_run:
        for s in segs:
            print(f"{s[0]:7.2f}-{s[1]:7.2f}s  {s[2]:9s} step={s[3]:2d} target={s[4]:+.2f}")
        print(f"total {total:.1f} s")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = Path(args.out) / (stamp + (f"_{args.label}" if args.label else ""))
    outdir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = Runner(args)

    def on_signal(signum, _frame):
        node.abort = True
        node.abort_reason = node.abort_reason or f"signal {signum}"
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    status = "completed"
    try:
        ok, info, problems = preflight(node, args)
        if not ok:
            status = "preflight_failed"
            print("PREFLIGHT FAILED:")
            for p in problems:
                print("  -", p)
            return 2
        print("Preflight OK:", json.dumps(info))
        ctrl_params = read_controller_params()
        print("diff_drive_controller params:", json.dumps(ctrl_params))
        print(f"Starting {len(args.steps)} steps, ~{total:.0f} s. Ctrl-C = immediate stop.")

        node.rows["cmd"].clear()
        # param reads above block without spinning: refresh the cached sim clock
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.02)
        t_start = node.now()
        period = 1.0 / args.rate
        next_pub = t_start
        last_clock, last_wall = node.now(), time.time()
        while not node.abort:
            rclpy.spin_once(node, timeout_sec=0.005)
            t = node.now()
            if t > last_clock:
                last_clock, last_wall = t, time.time()
            elif time.time() - last_wall > 2.0:
                node.abort, node.abort_reason = True, "sim clock stalled"
                break
            if node.gt_origin and node.latest_gt:
                d = math.hypot(node.latest_gt[0] - node.gt_origin[0],
                               node.latest_gt[1] - node.gt_origin[1])
                if d > args.max_drift:
                    node.abort, node.abort_reason = True, f"ground-truth drift {d:.2f} m"
                    break
            if t < next_pub:
                continue
            next_pub = max(next_pub + period, t)   # never burst-catch-up
            c = cmd_at(segs, t - t_start)
            if c is None:
                break
            wz, phase, idx, target = c
            node.send(wz)
            node.rows["cmd"].append((t, idx, phase, target, wz))
        if node.abort:
            status = f"aborted: {node.abort_reason}"
    finally:
        try:
            node.hard_stop()
            for _ in range(20):                       # collect the tail
                rclpy.spin_once(node, timeout_sec=0.02)
        except Exception:                             # noqa: BLE001
            pass
        write_csvs(node, outdir)
        meta = {
            "status": status,
            "args": vars(args),
            "schedule": segs,
            "controller_params": locals().get("ctrl_params"),
            "preflight": locals().get("info"),
            "sources": {
                "cmd": CMD_TOPIC, "cmd_out": CMD_OUT_TOPIC, "gt": args.gt_topic,
                "odom_raw": RAW_ODOM_TOPIC, "joints": JOINT_TOPIC,
                "imu": IMU_TOPIC, "odom_fused": FUSED_ODOM_TOPIC},
            "sample_counts": {k: len(v) for k, v in node.rows.items()},
            "sha256": {p: sha256(REPO / p) for p in (
                "src/amr_control/config/controllers.yaml",
                "src/amr_localization/config/ekf.yaml",
                "src/amr_navigation/config/nav2_params.yaml",
                "src/amr_description/urdf/amr.urdf.xacro")},
            "git_rev": sh(["git", "-C", str(REPO), "rev-parse", "HEAD"])[1],
            "time_base": "all t_recv/t_stamp/t columns are Gazebo sim time [s]",
            "wall_time": datetime.now().isoformat(),
        }
        (outdir / "metadata.json").write_text(json.dumps(meta, indent=2, default=str))
        print(f"{status}; data in {outdir}")
        print("sample counts:", meta["sample_counts"])
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
