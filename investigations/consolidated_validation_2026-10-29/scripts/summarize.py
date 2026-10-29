#!/usr/bin/env python3
"""Post-process the four primary runs: reverse segments, clearance, initial rotation, rear-obstacle evidence -> runs/summary.json + figures.
usage: summarize.py   (run from anywhere)"""
import json, subprocess, os, sys
root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')); V = f'{root}/investigations/consolidated_validation_2026-10-29'
NS = f'{root}/investigations/navfn_mppi_evaluation_2026-10-26/scripts'
runs = {'T1_routed_B_to_Astaging': None, 'T2_free_B_to_Home': None, 'T3_rear_obstacle': '-7.42 2.30 0.30', 'T4_routed_Home_to_B_yaw90': None}
out = {}
for r, box in runs.items():
    d = f'{V}/runs/{r}'
    if not os.path.exists(f'{d}/gt.csv'): continue
    sh = lambda *a: json.loads(subprocess.run(a, capture_output=True, text=True).stdout.strip().splitlines()[-1])
    seg = sh('python3', f'{NS}/reverse_segments.py', f'{d}/gt.csv', f'{d}/reverse_segments.json', f'{root}/docs/figures/val_{r}.png', r)
    cl = sh('python3', f'{V}/scripts/clearance.py', f'{d}/gt.csv', f'{d}/clearance.json', *(box.split() if box else []))
    ir = sh('python3', f'{NS}/initial_rotation.py', f'{d}/gt.csv', f'{d}/plog.jsonl')
    res = json.load(open(f'{d}/result.json'))[0]; m = json.load(open(f'{d}/metrics.json'))
    o = {'status': res.get('status'), 'nav2_error_code': res.get('nav2_error_code'), 'accepted': res.get('accepted'), 'timed_out_and_cancelled': res.get('timed_out_and_cancelled'),
         'metrics': m, 'reverse': {k: v for k, v in seg.items()}, 'clearance': cl, 'initial_rotation': ir}
    if os.path.exists(f'{d}/rear.jsonl'):
        L = [json.loads(l) for l in open(f'{d}/rear.jsonl')]
        sc = [x['v'] for x in L if x['topic'] == 'scan_rear_min_range_m' and x['v'] is not None]
        cm = [x for x in L if x['topic'] == 'local_costmap_box_region']
        cs = [x for x in L if x['topic'] == 'collision_monitor_state']
        nav = [x for x in L if x['topic'] == 'cmd_vel_nav']; sm = [x for x in L if x['topic'] == 'cmd_vel']
        o['rear'] = {'scan_rear_min_range_m_first_last_min': [sc[0], sc[-1], min(sc)] if sc else None,
                     'costmap_box_max_cost_max': max((x['max_cost'] for x in cm), default=None), 'costmap_box_centre_cost_max': max((x['centre_cost'] or 0 for x in cm), default=None),
                     'collision_monitor_states': sorted({(x['action_type'], x['polygon']) for x in cs}),
                     'cmd_vel_nav_vx_min': min((x['vx'] for x in nav), default=None), 'cmd_vel_final_vx_min': min((x['vx'] for x in sm), default=None)}
    out[r] = o
json.dump(out, open(f'{V}/runs/summary.json', 'w'), indent=1); print(json.dumps(out, indent=1)[:6000])
