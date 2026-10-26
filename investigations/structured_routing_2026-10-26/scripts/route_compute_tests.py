#!/usr/bin/env python3
"""Test A: graph/route computation checks against the running Route Server (ComputeRoute action).

usage: route_compute_tests.py OUT_JSON
Does not move the robot.
"""
import json
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputeRoute
from geometry_msgs.msg import PoseStamped

ERR = {0: 'NONE', 400: 'UNKNOWN', 401: 'TF_ERROR', 402: 'NO_VALID_GRAPH', 403: 'INDETERMINANT_NODES_ON_GRAPH',
       404: 'TIMEOUT', 405: 'NO_VALID_ROUTE', 407: 'INVALID_EDGE_SCORER_USE'}
N = {'HOME': 1, 'LANE_W': 2, 'LANE_A_JCT': 3, 'A_APPROACH': 4, 'A_STAGING': 5, 'STATION_B': 6}
XY = {'HOME': (-7.589, 1.2), 'STATION_B': (8.716, 2.705), 'A_STAGING': (0.099, -0.175)}


def pose(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    node = Node('route_compute_tests')
    cli = ActionClient(node, ComputeRoute, 'compute_route')
    assert cli.wait_for_server(timeout_sec=10.0), 'compute_route not available'

    def call(label, **kw):
        g = ComputeRoute.Goal()
        for k, v in kw.items():
            setattr(g, k, v)
        gh_f = cli.send_goal_async(g)
        rclpy.spin_until_future_complete(node, gh_f, timeout_sec=10)
        gh = gh_f.result()
        if not gh.accepted:
            return {'label': label, 'accepted': False}
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=10)
        r = rf.result().result
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
        L = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)) if len(pts) > 1 else 0.0
        out = {
            'label': label, 'error_code': int(r.error_code), 'error': ERR.get(int(r.error_code), '?'),
            'nodes': [n.nodeid for n in r.route.nodes], 'edges': [e.edgeid for e in r.route.edges],
            'route_cost': round(r.route.route_cost, 2), 'path_points': len(pts), 'path_length_m': round(L, 2),
            'planning_time_ms': round(r.planning_time.sec * 1e3 + r.planning_time.nanosec / 1e6, 2),
        }
        if r.path.poses:
            q = r.path.poses[-1].pose.orientation
            out['path_start_xy'] = [round(c, 3) for c in pts[0]]
            out['path_end_xy'] = [round(c, 3) for c in pts[-1]]
            out['path_end_yaw_deg'] = round(math.degrees(2 * math.atan2(q.z, q.w)), 1)
        return out

    res = []
    # By node id
    res.append(call('A1 id HOME->STATION_B', start_id=N['HOME'], goal_id=N['STATION_B'], use_start=False, use_poses=False))
    res.append(call('A2 id STATION_B->A_STAGING', start_id=N['STATION_B'], goal_id=N['A_STAGING'], use_poses=False))
    res.append(call('A3 id HOME->A_STAGING', start_id=N['HOME'], goal_id=N['A_STAGING'], use_poses=False))
    res.append(call('A4 id STATION_B->HOME (reverse lane direction)', start_id=N['STATION_B'], goal_id=N['HOME'], use_poses=False))
    # Directed-edge enforcement: no edge leaves A_STAGING
    res.append(call('A5 id A_STAGING->STATION_B (no outbound edge: structurally forbidden)', start_id=N['A_STAGING'], goal_id=N['STATION_B'], use_poses=False))
    res.append(call('A6 id A_APPROACH->LANE_A_JCT (spur is one-way inbound)', start_id=N['A_APPROACH'], goal_id=N['LANE_A_JCT'], use_poses=False))
    res.append(call('A7 id unknown goal node 99', start_id=N['HOME'], goal_id=99, use_poses=False))
    # By pose (start given explicitly so no TF dependency)
    s = pose(*XY['HOME']); g = pose(*XY['STATION_B'])
    res.append(call('A8 pose HOME->STATION_B', start=s, goal=g, use_start=True, use_poses=True))
    res.append(call('A9 pose HOME->A_STAGING', start=s, goal=pose(*XY['A_STAGING']), use_start=True, use_poses=True))
    # Off-graph goal inside a rack: server snaps to nearest node (documented behaviour)
    res.append(call('A10 pose HOME-> off-graph point (-1.5,-3.8) inside rack', start=s, goal=pose(-1.5, -3.8), use_start=True, use_poses=True))
    for r in res:
        print(json.dumps(r))
    json.dump(res, open(sys.argv[1], 'w'), indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
