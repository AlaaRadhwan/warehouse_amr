#!/usr/bin/env python3
"""Per-goal controller-behaviour metrics from Gazebo ground truth + /cmd_vel_nav log.

usage: analyze_goal.py GT.csv PLANLOG.jsonl GOAL_X GOAL_Y GOAL_YAW OUT.json [ROUTE_EDGE_IDS] [GRAPH.geojson]
Metrics (all from the first motion to the end of the recording):
  travelled_m, duration_s (motion start -> last sample), final position / yaw error (GT),
  reverse_gt_m   : distance travelled while GT velocity points backwards w.r.t. the body heading (|v|>0.03 m/s)
  reverse_cmd_s  : time with commanded vx < -0.02 m/s (/cmd_vel_nav, MPPI+shim-free)
  inplace_rot_deg: accumulated |yaw change| while GT linear speed < 0.05 m/s (spinning in place)
  max_dev_route_m: max distance of GT path from the graph route polyline (when edges given)
  near_goal_time_s: time spent within 0.5 m of the goal before the recording ends (terminal dwell)
"""
import csv, json, math, sys
import numpy as np

gt, plog, gx, gy, gyaw, out = sys.argv[1:7]
gx, gy, gyaw = float(gx), float(gy), float(gyaw)
rows = list(csv.DictReader(open(gt)))
t = np.array([float(r['sim']) for r in rows]); x = np.array([float(r['x']) for r in rows]); y = np.array([float(r['y']) for r in rows])
yaw = np.unwrap(2 * np.arctan2([float(r['qz']) for r in rows], [float(r['qw']) for r in rows]))
moved = (np.hypot(x - x[0], y - y[0]) > 0.02) | (np.abs(yaw - yaw[0]) > 0.035)   # any motion, including a rotation in place
i0 = int(np.argmax(moved)) if moved.any() else 0
# moving analysis on 0.2 s decimation
idx = [i0]
for i in range(i0, len(t)):
    if t[i] - t[idx[-1]] >= 0.2: idx.append(i)
idx = np.array(idx); tt, xx, yy, yw = t[idx], x[idx], y[idx], yaw[idx]
dt = np.diff(tt); dx = np.diff(xx); dy = np.diff(yy); dyw = np.diff(yw)
ds = np.hypot(dx, dy); sp = ds / np.maximum(dt, 1e-6)
head = (yw[:-1] + yw[1:]) / 2
along = (dx * np.cos(head) + dy * np.sin(head))
rev = along < 0
res = {
    'travelled_m': round(float(ds.sum()), 2),
    'duration_s': round(float(t[-1] - t[i0]), 1),
    'final_xy': [round(float(x[-1]), 3), round(float(y[-1]), 3)],
    'final_position_error_m': round(float(math.hypot(x[-1] - gx, y[-1] - gy)), 3),
    'final_yaw_error_deg': round(math.degrees((yaw[-1] - gyaw + math.pi) % (2 * math.pi) - math.pi), 1),
    'reverse_gt_m': round(float(ds[rev & (sp > 0.03)].sum()), 2),
    'inplace_rot_deg': round(math.degrees(float(np.abs(dyw[sp < 0.05]).sum())), 0),
    'total_rot_deg': round(math.degrees(float(np.abs(dyw).sum())), 0),
}
d_goal = np.hypot(xx - gx, yy - gy)
res['near_goal_time_s'] = round(float(dt[d_goal[:-1] < 0.5].sum()), 1)
try:
    cmd = [json.loads(l) for l in open(plog)]
    cmd = [c for c in cmd if c['topic'] == 'cmd_vel_nav']
    res['reverse_cmd_s'] = round(0.05 * sum(1 for c in cmd if c['vx'] < -0.02), 1)
    res['cmd_vx_min'] = round(min((c['vx'] for c in cmd), default=0), 3)
except FileNotFoundError:
    pass
if len(sys.argv) > 8:
    ids = [int(e) for e in sys.argv[7].split(',')]
    g = json.load(open(sys.argv[8])); E = {f['properties']['id']: f['geometry']['coordinates'][0] for f in g['features'] if 'startid' in f['properties']}
    poly = [np.array(E[e][0]) for e in ids] + [np.array(E[ids[-1]][1])]
    segs = list(zip(poly[:-1], poly[1:]))
    def dseg(p, a, b):
        ab = b - a; s = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1); return np.linalg.norm(p - (a + s * ab))
    dev = [min(dseg(np.array([a, b]), s0, s1) for s0, s1 in segs) for a, b in zip(xx, yy)]
    res['max_dev_route_m'] = round(float(max(dev)), 2); res['mean_dev_route_m'] = round(float(np.mean(dev)), 2)
json.dump(res, open(out, 'w'), indent=1); print(json.dumps(res))
