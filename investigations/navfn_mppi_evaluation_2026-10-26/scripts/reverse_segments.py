#!/usr/bin/env python3
"""Reverse-segment statistics + trajectory figure from GT.
usage: reverse_segments.py GT.csv OUT.json [FIG.png TITLE]
A reverse sample = 0.2 s GT step with displacement opposite to body heading and speed > 0.03 m/s.
Samples separated by <= 1.0 s of non-reverse motion/stop are merged into one segment.
Also reports forward distance, and the initial-heading figures used elsewhere (see initial_rotation.py)."""
import csv, json, math, sys
import numpy as np
g = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r['sim']) for r in g]); x = np.array([float(r['x']) for r in g]); y = np.array([float(r['y']) for r in g])
yaw = np.unwrap(2 * np.arctan2([float(r['qz']) for r in g], [float(r['qw']) for r in g]))
mv = (np.hypot(x - x[0], y - y[0]) > 0.02) | (np.abs(yaw - yaw[0]) > 0.035); i0 = int(np.argmax(mv))
idx = [i0]
for i in range(i0, len(t)):
    if t[i] - t[idx[-1]] >= 0.2: idx.append(i)
idx = np.array(idx); tt, xx, yy, yw = t[idx], x[idx], y[idx], yaw[idx]
dx, dy = np.diff(xx), np.diff(yy); ds = np.hypot(dx, dy); dt = np.diff(tt)
head = (yw[:-1] + yw[1:]) / 2; along = dx * np.cos(head) + dy * np.sin(head); sp = ds / np.maximum(dt, 1e-6)
rev = (along < 0) & (sp > 0.03); fwd = (along >= 0) & (sp > 0.03)
segs, cur = [], None
for k in range(len(ds)):
    if rev[k]:
        if cur and tt[k] - cur['t1'] <= 1.0: cur['len'] += ds[k]; cur['t1'] = tt[k + 1]
        else:
            if cur: segs.append(cur)
            cur = {'t0': tt[k], 't1': tt[k + 1], 'len': ds[k], 'x0': xx[k], 'y0': yy[k]}
if cur: segs.append(cur)
lens = [s['len'] for s in segs]
res = {'forward_m': round(float(ds[fwd].sum()), 2), 'reverse_m': round(float(ds[rev].sum()), 2),
       'reverse_segments': len(segs), 'reverse_segments_ge_0.3m': int(sum(l >= 0.3 for l in lens)),
       'max_continuous_reverse_m': round(max(lens, default=0.0), 2),
       'segment_lengths_m': [round(l, 2) for l in lens],
       'segment_start_s_xy': [[round(s['t0'] - tt[0], 1), round(s['x0'], 2), round(s['y0'], 2)] for s in segs]}
json.dump(res, open(sys.argv[2], 'w'), indent=1)
print(json.dumps({k: v for k, v in res.items() if k != 'segment_start_s_xy'}))
if len(sys.argv) > 3:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(xx, yy, '-', color='#bbbbbb', lw=1, zorder=1)
    mx, my = (xx[:-1] + xx[1:]) / 2, (yy[:-1] + yy[1:]) / 2
    ax.scatter(mx[fwd], my[fwd], s=6, c='#1f77b4', label='forward', zorder=2)
    ax.scatter(mx[rev], my[rev], s=10, c='#d62728', label='reverse', zorder=3)
    st = ~(fwd | rev); ax.scatter(mx[st], my[st], s=10, c='#2ca02c', marker='x', label='in place / stopped', zorder=2)
    ax.plot(xx[0], yy[0], 'k^', ms=9, label='start'); ax.plot(xx[-1], yy[-1], 'ks', ms=8, label='end')
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f"{sys.argv[4] if len(sys.argv) > 4 else ''}\nforward {res['forward_m']} m, reverse {res['reverse_m']} m "
                 f"({res['reverse_segments']} segments, max continuous {res['max_continuous_reverse_m']} m)", fontsize=10)
    ax.legend(loc='best', fontsize=8); fig.tight_layout(); fig.savefig(sys.argv[3], dpi=110)
