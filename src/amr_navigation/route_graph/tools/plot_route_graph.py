#!/usr/bin/env python3
"""Plot the route graph over the saved occupancy map and report edge clearance.

    python3 plot_route_graph.py [--graph G.geojson] [--out fig.png] [--traj traj.csv ...]

Clearance = distance from each point on an edge to the nearest non-free map cell
(unknown cells count as blocked). The robot's circumscribed radius is 0.62 m.
"""
import argparse
import json
import os
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.abspath(os.path.join(HERE, '..', '..'))
CIRCUM_R = 0.62


def load_map():
    meta = yaml.safe_load(open(os.path.join(NAV, 'maps', 'warehouse.yaml')))
    a = np.array(Image.open(os.path.join(NAV, 'maps', meta['image'])))
    return a, float(meta['resolution']), meta['origin'][:2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', default=os.path.join(HERE, '..', 'warehouse_route_graph.geojson'))
    ap.add_argument('--out', default='route_graph.png')
    ap.add_argument('--traj', nargs='*', default=[], help='CSV files with columns t,x,y (optional overlay)')
    ap.add_argument('--xlim', nargs=2, type=float, default=[-11, 14])
    ap.add_argument('--ylim', nargs=2, type=float, default=[-5, 7])
    args = ap.parse_args()

    img, res, (ox, oy) = load_map()
    h, w = img.shape
    clear = ndi.distance_transform_edt(img == 254) * res

    def cl(x, y):
        return clear[h - 1 - int((y - oy) / res), int((x - ox) / res)]

    g = json.load(open(args.graph))
    nodes, edges = {}, []
    for f in g['features']:
        p = f['properties']
        if 'startid' in p:
            edges.append((p, f['geometry']['coordinates'][0]))
        else:
            nodes[p['id']] = (f['geometry']['coordinates'], p['metadata'])

    fig, ax = plt.subplots(figsize=(15, 7.5))
    ax.imshow(img, extent=[ox, ox + w * res, oy, oy + h * res], cmap='gray', origin='upper')
    print('edge clearance (m, min along edge; robot circumscribed radius %.2f):' % CIRCUM_R)
    pair = {}
    for p, (s, e) in edges:
        pair[(p['startid'], p['endid'])] = (s, e)
    for p, (s, e) in edges:
        s, e = np.array(s), np.array(e)
        L = np.linalg.norm(e - s)
        pts = [s + t * (e - s) for t in np.linspace(0, 1, max(2, int(L / 0.05)))]
        mc = min(cl(*q) for q in pts)
        two_way = (p['endid'], p['startid']) in pair
        d = (e - s) / L
        off = np.array([-d[1], d[0]]) * (0.12 if two_way else 0.0)
        s2, e2 = s + off, e + off
        ax.annotate('', xy=e2, xytext=s2, arrowprops=dict(
            arrowstyle='-|>', color='tab:blue' if two_way else 'tab:red', lw=2, mutation_scale=16))
        mid = (s2 + e2) / 2 + np.array([-d[1], d[0]]) * (0.3 if two_way else 0.3)
        ax.text(*mid, f"e{p['id']}", fontsize=8, color='navy', ha='center')
        print(f"  e{p['id']:<4}{p['metadata']['name']:<26} len {L:6.2f}  min clearance {mc:5.2f}"
              f"  {'OK' if mc > CIRCUM_R else 'TOO NARROW'}  {'two-way' if two_way else 'ONE-WAY'}")
    for nid, ((x, y), md) in nodes.items():
        ax.plot(x, y, 'o', ms=11, color='gold', mec='k', zorder=5)
        ax.text(x, y + 0.35, f"n{nid} {md['name']}", fontsize=9, ha='center', weight='bold', color='k',
                bbox=dict(fc='w', ec='none', alpha=.7, pad=1))
        print(f"  n{nid:<3}{md['name']:<14} ({x:7.3f},{y:7.3f}) clearance {cl(x, y):4.2f}")
    for tf in args.traj:
        d = np.genfromtxt(tf, delimiter=',', names=True)
        ax.plot(d['x'], d['y'], '-', lw=1.5, label=os.path.basename(tf))
    if args.traj:
        ax.legend(loc='lower right')
    ax.plot([], [], '-', color='tab:blue', label='two-way lane (one directed edge each way)')
    ax.plot([], [], '-', color='tab:red', label='one-way edge')
    ax.legend(loc='upper left', fontsize=8)
    ax.set_xlim(*args.xlim); ax.set_ylim(*args.ylim); ax.set_aspect('equal')
    ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)'); ax.grid(alpha=.3)
    ax.set_title('Warehouse route graph over saved occupancy map (white=free, gray=unknown, black=occupied)')
    fig.savefig(args.out, dpi=100, bbox_inches='tight')
    print('wrote', args.out)


if __name__ == '__main__':
    main()
