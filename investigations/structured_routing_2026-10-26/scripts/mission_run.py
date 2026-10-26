#!/usr/bin/env python3
"""Send GoToStation goals (existing amr_missions interface) with finite time limits and record evidence.

usage: mission_run.py OUT_JSON GOAL[:TIMEOUT_S] [GOAL[:TIMEOUT_S] ...]
GOAL = station name (GoToStation) or  nav@X@Y@YAW[@bt|@default]  (direct NavigateToPose; bt = route BT).
For each goal: result, nav2 error code, feedback samples, sim/wall duration, the first
/received_global_plan (the path the controller was actually given), real-time factor.
A goal that exceeds its timeout is cancelled (never left running).
"""
import json
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from amr_interfaces.action import GoToStation
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math, os
from ament_index_python.packages import get_package_share_directory


class Runner(Node):
    def __init__(self):
        super().__init__('mission_run')
        self.cli = ActionClient(self, GoToStation, 'mission/go_to_station')
        self.sim = 0.0
        self.plan = None
        self.plans = 0
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim', m.clock.sec + m.clock.nanosec * 1e-9), q)
        self.create_subscription(Path, '/received_global_plan', self.on_plan, q)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_plan(self, m):
        self.plans += 1
        if self.plan is None and m.poses:
            self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def go_nav(self, name, g, timeout, w0, s0):
        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        gh = fut.result()
        rec = {'station': name, 'accepted': bool(gh and gh.accepted), 'timeout_s': timeout, 'behavior_tree': g.behavior_tree}
        if not rec['accepted']:
            return rec
        rf = gh.get_result_async()
        cancelled = False
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - w0 > timeout and not cancelled:
                gh.cancel_goal_async(); cancelled = True
            if time.time() - w0 > timeout + 30:
                break
        res = rf.result()
        rec.update({'timed_out_and_cancelled': cancelled, 'status': 'SUCCEEDED' if res and res.status == GoalStatus.STATUS_SUCCEEDED else str(res.status if res else 'NO_RESULT'),
                    'success': bool(res and res.status == GoalStatus.STATUS_SUCCEEDED), 'nav2_error_code': int(res.result.error_code) if res else -1,
                    'message': res.result.error_msg if res else '', 'sim_duration_s': round(self.sim - s0, 2), 'wall_duration_s': round(time.time() - w0, 2)})
        rec['rtf'] = round(rec['sim_duration_s'] / max(rec['wall_duration_s'], 1e-6), 3)
        rec['plan_points'] = len(self.plan) if self.plan else 0
        rec['plans_received'] = self.plans
        rec['plan_xy'] = [[round(x, 3), round(y, 3)] for x, y in (self.plan[::10] if self.plan else [])]
        return rec

    def go(self, station, timeout):
        self.plan, self.plans = None, 0
        w0, s0 = time.time(), self.sim
        if station.startswith('nav@'):
            _, x, y, yaw, *mode = station.split('@')
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = 'map'
            g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(float(yaw) / 2), math.cos(float(yaw) / 2)
            if mode and mode[0] == 'bt':
                g.behavior_tree = os.path.join(get_package_share_directory('amr_navigation'), 'behavior_trees', 'navigate_on_warehouse_route.xml')
            return self.go_nav(station, g, timeout, w0, s0)
        g = GoToStation.Goal()
        g.station_name = station
        fut = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        gh = fut.result()
        rec = {'station': station, 'accepted': bool(gh and gh.accepted), 'timeout_s': timeout}
        if not rec['accepted']:
            return rec
        rf = gh.get_result_async()
        cancelled = False
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - w0 > timeout and not cancelled:
                self.get_logger().error(f'{station}: wall timeout {timeout}s -> cancelling')
                gh.cancel_goal_async()
                cancelled = True
            if time.time() - w0 > timeout + 30:
                break
        res = rf.result()
        rec.update({
            'timed_out_and_cancelled': cancelled,
            'status': GoalStatus.STATUS_SUCCEEDED == (res.status if res else -1) and 'SUCCEEDED' or str(res.status if res else 'NO_RESULT'),
            'success': bool(res and res.result.success),
            'message': res.result.message if res else '',
            'nav2_error_code': int(res.result.nav2_error_code) if res else -1,
            'sim_duration_s': round(self.sim - s0, 2), 'wall_duration_s': round(time.time() - w0, 2),
        })
        rec['rtf'] = round(rec['sim_duration_s'] / max(rec['wall_duration_s'], 1e-6), 3)
        rec['plan_points'] = len(self.plan) if self.plan else 0
        rec['plans_received'] = self.plans
        rec['sim_stamp_end'] = round(self.sim, 2)
        rec['plan_xy'] = [[round(x, 3), round(y, 3)] for x, y in (self.plan[::10] if self.plan else [])]
        return rec


def main():
    out = sys.argv[1]
    goals = [(a.rsplit(':', 1)[0], float(a.rsplit(':', 1)[1])) if ':' in a else (a, 120.0) for a in sys.argv[2:]]
    rclpy.init()
    r = Runner()
    assert r.cli.wait_for_server(timeout_sec=20), 'mission/go_to_station not available'
    r.spin_for(2.0)
    results = []
    for st, to in goals:
        rec = r.go(st, to)
        print(json.dumps({k: v for k, v in rec.items() if k != 'plan_xy'}))
        results.append(rec)
        r.spin_for(3.0)  # settle
        json.dump(results, open(out, 'w'), indent=1)
        if not rec.get('success'):
            break
    r.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
