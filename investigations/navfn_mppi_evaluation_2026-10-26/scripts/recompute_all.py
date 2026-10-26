#!/usr/bin/env python3
"""Recompute metrics.json for every run directory from its raw gt.csv/plog.jsonl (after a metric change)."""
import glob, json, os, re, subprocess, sys
H = os.path.dirname(os.path.abspath(__file__)); RUNS = os.path.join(H, '..', 'runs'); ROOT = os.path.join(H, '..', '..', '..')
G = os.path.join(ROOT, 'src/amr_navigation/route_graph/warehouse_route_graph.geojson'); A = os.path.join(H, 'analyze_goal.py')
EDGES = {'R1': '101,103,105', 'R2': '106,107,108', 'Ryaw': '101,103,105'}
for d in sorted(glob.glob(os.path.join(RUNS, '*_[AR]*_*'))):
    if 'INVALID' in d or not os.path.exists(os.path.join(d, 'result.json')): continue
    res = json.load(open(os.path.join(d, 'result.json')))[0]
    m = re.match(r'nav@([-\d.]+)@([-\d.]+)@([-\d.]+)', res['station']); gx, gy, gyaw = m.groups()
    e = EDGES.get(os.path.basename(d).split('_')[1])
    cmd = [sys.executable, A, os.path.join(d, 'gt.csv'), os.path.join(d, 'plog.jsonl'), gx, gy, gyaw, os.path.join(d, 'metrics.json')] + ([e, G] if e else [])
    subprocess.run(cmd, check=True, capture_output=True)
# baselines (State Lattice + baseline MPPI) from the structured-routing runs
SR = os.path.join(ROOT, 'investigations/structured_routing_2026-10-26/runs'); O = os.path.join(RUNS, 'baseline_recomputed')
for name, gt, plog, goal, e in [
    ('lattice_free_Home_to_B', 'testE_default_gt.csv', None, (8.716, 2.705, 0), None),
    ('lattice_free_B_to_Astaging', 'testF_default_BtoA_gt.csv', None, (0.099, -0.175, 0), None),
    ('lattice_route_Home_to_B', 'testB_gt.csv', None, (8.716, 2.705, 0), '101,103,105'),
    ('lattice_route_B_to_Astaging_run1', 'testC1_gt.csv', None, (0.099, -0.175, 0), '106,107,108'),
    ('lattice_route_B_to_Astaging_run2', 'testC2_gt.csv', None, (0.099, -0.175, 0), '106,107,108')]:
    cmd = [sys.executable, A, os.path.join(SR, gt), '/nonexistent', *map(str, goal), os.path.join(O, name + '.json')] + ([e, G] if e else [])
    subprocess.run(cmd, check=True, capture_output=True)
print('recomputed')
