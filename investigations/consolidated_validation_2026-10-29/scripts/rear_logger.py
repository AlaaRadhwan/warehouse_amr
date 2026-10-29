#!/usr/bin/env python3
"""Rear-obstacle evidence logger (Test 3). usage: rear_logger.py OUT.jsonl BOX_X BOX_Y BOX_HALF   (box centre/half-size in map frame)
Logs (sim time): /scan min range in a +-40 deg sector around the rear direction; local-costmap max / centre cost over the box
region; /collision_monitor_state; cmd_vel_nav (MPPI), cmd_vel_smoothed and cmd_vel (after collision monitor) vx/wz."""
import json, math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CollisionMonitorState
from geometry_msgs.msg import TwistStamped, PointStamped
from rosgraph_msgs.msg import Clock
import tf2_ros, tf2_geometry_msgs  # noqa

bx, by, bh = map(float, sys.argv[2:5])
rclpy.init(); n = Node('rear_logger'); f = open(sys.argv[1], 'w'); sim = [0.0]
qb = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
def w(d): d['t'] = round(sim[0], 2); f.write(json.dumps(d) + '\n'); f.flush()
n.create_subscription(Clock, '/clock', lambda m: sim.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9), qb)
ls = [0]
def scan(m):
    if sim[0] - ls[0] < 0.5: return
    ls[0] = sim[0]; best = None
    for i, r in enumerate(m.ranges):
        a = m.angle_min + i * m.angle_increment; a = math.atan2(math.sin(a - math.pi), math.cos(a - math.pi))
        if abs(a) <= math.radians(40) and m.range_min < r < m.range_max and (best is None or r < best): best = r
    w({'topic': 'scan_rear_min_range_m', 'v': None if best is None else round(best, 3)})
n.create_subscription(LaserScan, '/scan', scan, qb)
lc = [0]
def costmap(m):
    if sim[0] - lc[0] < 0.5: return
    lc[0] = sim[0]
    try: T = buf.lookup_transform(m.header.frame_id, 'map', rclpy.time.Time(), Duration(seconds=0.2))
    except Exception: return
    t = T.transform.translation; yaw = 2 * math.atan2(T.transform.rotation.z, T.transform.rotation.w)
    c, s = math.cos(yaw), math.sin(yaw); res = m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
    best = -1; centre = None
    for dx in [k * res for k in range(int(-bh / res), int(bh / res) + 1)]:
        for dy in [k * res for k in range(int(-bh / res), int(bh / res) + 1)]:
            X, Y = bx + dx, by + dy; px, py = c * X - s * Y + t.x, s * X + c * Y + t.y
            ix, iy = int((px - ox) / res), int((py - oy) / res)
            if 0 <= ix < m.info.width and 0 <= iy < m.info.height:
                v = m.data[iy * m.info.width + ix]; best = max(best, v)
                if abs(dx) < res and abs(dy) < res: centre = v
    w({'topic': 'local_costmap_box_region', 'max_cost': best, 'centre_cost': centre})
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', costmap, QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(CollisionMonitorState, '/collision_monitor_state',
    lambda m: w({'topic': 'collision_monitor_state', 'action_type': m.action_type, 'polygon': m.polygon_name}), 10)
for name, topic in (('cmd_vel_nav', '/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel_smoothed'), ('cmd_vel', '/cmd_vel')):
    last = [0.0]
    def mk(name, last):
        def cb(m):
            if sim[0] - last[0] >= 0.1:
                last[0] = sim[0]; w({'topic': name, 'vx': round(m.twist.linear.x, 3), 'wz': round(m.twist.angular.z, 3)})
        return cb
    n.create_subscription(TwistStamped, topic, mk(name, last), qb)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
