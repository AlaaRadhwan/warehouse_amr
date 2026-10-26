#!/usr/bin/env python3
"""Quantify route adherence and goal error from a Gazebo ground-truth CSV.

usage: analyze_route_run.py GT_CSV GRAPH_GEOJSON EDGE_IDS(comma) GOAL_X GOAL_Y GOAL_YAW OUT_PREFIX [T_START_WALL]
Writes OUT_PREFIX_traj.csv (t,x,y subsampled) and prints a JSON summary:
 - lateral deviation of the ground-truth path from the planned graph route polyline (max / p95 / mean)
 - final ground-truth position and yaw error vs goal
 - travelled path length vs route length
"""
import csv
import json
import math
import sys

import numpy as np

gt, graph, edge_ids, gx, gy, gyaw, prefix = sys.argv[1:8]
edge_ids = [int(e) for e in edge_ids.split(',')]
gx, gy, gyaw = float(gx), float(gy), float(gyaw)

g = json.load(open(graph))
nodes, edges = {}, {}
for f in g['features']:
    p = f['properties']
    if 'startid' in p:
        edges[p['id']] = f['geometry']['coordinates'][0]
poly = [np.array(edges[e][0]) for e in edge_ids] + [np.array(edges[edge_ids[-1]][1])]
segs = [(poly[i], poly[i + 1]) for i in range(len(poly) - 1)]
route_len = sum(np.linalg.norm(b - a) for a, b in segs)

rows = list(csv.DictReader(open(gt)))
t = np.array([float(r['sim']) for r in rows])
x = np.array([float(r['x']) for r in rows])
y = np.array([float(r['y']) for r in rows])
qz = np.array([float(r['qz']) for r in rows]); qw = np.array([float(r['qw']) for r in rows])
yaw = 2 * np.arctan2(qz, qw)

# moving segment: from first displacement > 2 cm to last
d0 = np.hypot(x - x[0], y - y[0])
i0 = int(np.argmax(d0 > 0.02))
sp = np.hypot(np.diff(x), np.diff(y))
i1 = len(x) - 1


def dist_to_seg(p, a, b):
    ab = b - a
    s = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1)
    return np.linalg.norm(p - (a + s * ab))


dev = np.array([min(dist_to_seg(np.array([xx, yy]), a, b) for a, b in segs) for xx, yy in zip(x[i0:i1 + 1], y[i0:i1 + 1])])
travel = float(sp[i0:].sum())
yerr = math.degrees((yaw[-1] - gyaw + math.pi) % (2 * math.pi) - math.pi)
out = {
    'route_length_m': round(float(route_len), 2),
    'travelled_length_m': round(travel, 2),
    'motion_start_sim_s': round(float(t[i0]), 2),
    'last_sample_sim_s': round(float(t[-1]), 2),
    'lateral_deviation_from_route_m': {'max': round(float(dev.max()), 3), 'p95': round(float(np.percentile(dev, 95)), 3), 'mean': round(float(dev.mean()), 3)},
    'final_gt_xy': [round(float(x[-1]), 3), round(float(y[-1]), 3)],
    'final_position_error_m': round(float(math.hypot(x[-1] - gx, y[-1] - gy)), 3),
    'final_yaw_error_deg': round(yerr, 1),
}
step = max(1, len(x) // 400)
with open(prefix + '_traj.csv', 'w') as f:
    f.write('t,x,y\n')
    for k in range(0, len(x), step):
        f.write(f'{t[k]:.2f},{x[k]:.4f},{y[k]:.4f}\n')
print(json.dumps(out, indent=1))
json.dump(out, open(prefix + '_metrics.json', 'w'), indent=1)
