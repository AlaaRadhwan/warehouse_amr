#!/usr/bin/env python3
"""Initial heading correction from GT: yaw change and duration from first yaw motion until the robot has moved 0.30 m from its start. usage: initial_rotation.py gt.csv [plog.jsonl]"""
import csv, json, math, sys
import numpy as np
g = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r['sim']) for r in g]); x = np.array([float(r['x']) for r in g]); y = np.array([float(r['y']) for r in g])
yaw = np.unwrap(2 * np.arctan2([float(r['qz']) for r in g], [float(r['qw']) for r in g]))
mv = (np.hypot(x - x[0], y - y[0]) > 0.02) | (np.abs(yaw - yaw[0]) > 0.035); i0 = int(np.argmax(mv))
i1 = int(np.argmax(np.hypot(x - x[0], y - y[0]) > 0.30))
out = {'start_yaw_deg': round(math.degrees(yaw[0]), 1), 'yaw_at_0.3m_deg': round(math.degrees(yaw[i1]), 1), 'rotation_before_0.3m_deg': round(math.degrees(abs(yaw[i1] - yaw[i0])), 0), 'time_motion_start_to_0.3m_s': round(float(t[i1] - t[i0]), 1)}
if len(sys.argv) > 2:
    p = [json.loads(l) for l in open(sys.argv[2])]; p = [x for x in p if x['topic'] == '/plan']
    out['plan_msgs'] = len(p); out['plan_rev_m_max'] = max((x.get('plan_rev_m', 0) for x in p), default=None); out['plan_cusps_max'] = max((x.get('plan_cusps', 0) for x in p), default=None)
print(json.dumps(out))
