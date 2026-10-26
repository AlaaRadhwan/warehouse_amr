#!/usr/bin/env python3
"""Log every /plan and /transformed_global_plan message summary (sim time, #poses, first/last xy+yaw) and /cmd_vel_nav to CSV-ish JSONL.
usage: plan_logger.py OUT.jsonl  (SIGINT to stop)"""
import json, math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped

rclpy.init(); n = Node('plan_logger'); f = open(sys.argv[1], 'w'); sim = [0.0]
q = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Clock, '/clock', lambda m: sim.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9), q)
def yaw(p): o = p.pose.orientation; return round(math.degrees(2 * math.atan2(o.z, o.w)), 1)
def mk(topic):
    def cb(m):
        if not m.poses: return
        a, b = m.poses[0], m.poses[-1]
        # plan geometry: reverse = step opposite to the pose heading; cusps = direction reversals between steps
        rev_len = fwd_len = 0.0; cusps = 0; inplace = 0; prev = None
        for p0, p1 in zip(m.poses[:-1], m.poses[1:]):
            dx = p1.pose.position.x - p0.pose.position.x; dy = p1.pose.position.y - p0.pose.position.y; d = math.hypot(dx, dy)
            h = 2 * math.atan2(p0.pose.orientation.z, p0.pose.orientation.w)
            if d < 1e-4: inplace += 1; continue
            fw = (dx * math.cos(h) + dy * math.sin(h)) > 0
            rev_len += 0.0 if fw else d; fwd_len += d if fw else 0.0
            if prev is not None and prev != fw: cusps += 1
            prev = fw
        f.write(json.dumps({'t': round(sim[0], 2), 'topic': topic, 'n': len(m.poses),
          'plan_fwd_m': round(fwd_len, 2), 'plan_rev_m': round(rev_len, 2), 'plan_cusps': cusps, 'plan_inplace_poses': inplace,
          'first': [round(a.pose.position.x, 2), round(a.pose.position.y, 2), yaw(a)],
          'last': [round(b.pose.position.x, 2), round(b.pose.position.y, 2), yaw(b)]}) + '\n'); f.flush()
    return cb
for t in ('/plan', '/transformed_global_plan'):
    n.create_subscription(Path, t, mk(t), q)
last = [0]
def cmd(m):
    if sim[0] - last[0] >= 0.05:
        last[0] = sim[0]; f.write(json.dumps({'t': round(sim[0], 2), 'topic': 'cmd_vel_nav', 'vx': round(m.twist.linear.x, 3), 'wz': round(m.twist.angular.z, 3)}) + '\n'); f.flush()
n.create_subscription(TwistStamped, '/cmd_vel_nav', cmd, q)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
