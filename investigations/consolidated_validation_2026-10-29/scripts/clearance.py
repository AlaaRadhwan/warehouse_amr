#!/usr/bin/env python3
"""Min footprint-to-obstacle clearance along a GT trajectory. usage: clearance.py GT.csv OUT.json [BOX_X BOX_Y BOX_HALF]
Obstacles = occupied/unknown cells of maps/warehouse.pgm (static map; NOT the person/low objects outside the map) plus an optional
axis-aligned box rasterised into the map. Footprint 1.0 x 0.74 m centred on base_footprint, boundary sampled every 2 cm and looked up
in a Euclidean distance transform (5 cm raster -> ~+-2.5 cm resolution). Reports min clearance and time/pose of the minimum."""
import csv, json, math, sys
import numpy as np
from PIL import Image
from scipy import ndimage as ndi
root = __file__.rsplit('/investigations/', 1)[0]
im = np.array(Image.open(f'{root}/src/amr_navigation/maps/warehouse.pgm')); free = im > 205
res, ox, oy = 0.05, -15.158, -25.231; H = im.shape[0]
if len(sys.argv) > 5:
    bx, by, bh = map(float, sys.argv[3:6])
    for x in np.arange(bx - bh, bx + bh + 1e-9, res / 2):
        for y in np.arange(by - bh, by + bh + 1e-9, res / 2):
            free[int(H - 1 - (y - oy) / res), int((x - ox) / res)] = False
dist = ndi.distance_transform_edt(free) * res
g = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r['sim']) for r in g]); x = np.array([float(r['x']) for r in g]); y = np.array([float(r['y']) for r in g])
yaw = 2 * np.arctan2([float(r['qz']) for r in g], [float(r['qw']) for r in g])
s = np.arange(-0.5, 0.5001, 0.02); u = np.arange(-0.37, 0.3701, 0.02)
bd = np.array([(a, 0.37) for a in s] + [(a, -0.37) for a in s] + [(0.5, b) for b in u] + [(-0.5, b) for b in u])
best = (9.0, 0, 0, 0, 0)
for i in range(0, len(t), 5):
    c, sn = math.cos(yaw[i]), math.sin(yaw[i])
    px = x[i] + c * bd[:, 0] - sn * bd[:, 1]; py = y[i] + sn * bd[:, 0] + c * bd[:, 1]
    r = np.clip((H - 1 - (py - oy) / res).astype(int), 0, H - 1); q = np.clip(((px - ox) / res).astype(int), 0, dist.shape[1] - 1)
    d = float(dist[r, q].min())
    if d < best[0]: best = (d, t[i], x[i], y[i], math.degrees(yaw[i]))
out = {'min_footprint_clearance_m': round(best[0], 3), 'at_sim_s': round(best[1], 1), 'at_xy_yaw_deg': [round(best[2], 2), round(best[3], 2), round(best[4], 1)],
       'obstacles': 'static map' + (' + box' if len(sys.argv) > 5 else ''), 'resolution_note': '5 cm raster'}
json.dump(out, open(sys.argv[2], 'w')); print(json.dumps(out))
