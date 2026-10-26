#!/usr/bin/env python3
"""Generate the warehouse route graph (Nav2 Route Server GeoJSON) from a compact table.

Single source of truth for node coordinates and edge directions.  Station
coordinates are read from amr_missions/config/stations.yaml so that the graph
and the mission configuration cannot drift apart.

    python3 make_route_graph.py [--out warehouse_route_graph.geojson]

All geometry assumptions are documented in docs/STRUCTURED_ROUTING.md.
"""
import argparse
import json
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_SRC = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
STATIONS = os.path.join(REPO_SRC, 'amr_missions', 'config', 'stations.yaml')
# Nav2 DockRobot staging pose is the final dock pose shifted by this x offset
# (docking_server.load_station_dock.staging_x_offset in nav2_params.yaml).
STAGING_X_OFFSET = -0.70

# Lane height shared with Station B so the final lane edge arrives at B with yaw 0.
LANE_Y = None  # set from station_b below


def build():
    with open(STATIONS) as f:
        st = yaml.safe_load(f)['stations']
    home, a, b = st['home'], st['station_a'], st['station_b']
    lane_y = float(b['y'])
    a_stage = (float(a['x']) + STAGING_X_OFFSET, float(a['y']))

    # id: (x, y, name, class)
    nodes = {
        1: (home['x'], home['y'], 'HOME', 'station'),
        2: (-5.5, lane_y, 'LANE_W', 'junction'),
        3: (-3.0, lane_y, 'LANE_A_JCT', 'junction'),
        4: (-3.0, a['y'], 'A_APPROACH', 'junction'),
        5: (a_stage[0], a_stage[1], 'A_STAGING', 'station_staging'),
        6: (b['x'], b['y'], 'STATION_B', 'station'),
    }

    # (id, from, to, kind). Two-way lane = two directed edges (one per direction).
    edges = [
        (101, 1, 2, 'lane'), (102, 2, 1, 'lane'),
        (103, 2, 3, 'lane'), (104, 3, 2, 'lane'),
        (105, 3, 6, 'lane'), (106, 6, 3, 'lane'),
        # Station A spur: one-way inbound, final approach heading +x (= dock yaw).
        (107, 3, 4, 'a_spur_inbound'),
        (108, 4, 5, 'a_spur_inbound'),
    ]
    return nodes, edges


def to_geojson(nodes, edges):
    feats = []
    for nid, (x, y, name, cls) in nodes.items():
        feats.append({
            'type': 'Feature',
            'properties': {'id': nid, 'frame': 'map',
                           'metadata': {'name': name, 'class': cls}},
            'geometry': {'type': 'Point', 'coordinates': [round(float(x), 3), round(float(y), 3)]},
        })
    for eid, s, e, kind in edges:
        (x0, y0), (x1, y1) = nodes[s][:2], nodes[e][:2]
        feats.append({
            'type': 'Feature',
            'properties': {
                'id': eid, 'startid': s, 'endid': e,
                'overridable': True,
                'metadata': {
                    'name': f'{nodes[s][2]}->{nodes[e][2]}',
                    'class': kind,
                    'length_m': round(math.hypot(x1 - x0, y1 - y0), 3),
                    'heading_deg': round(math.degrees(math.atan2(y1 - y0, x1 - x0)), 1),
                },
            },
            'geometry': {'type': 'MultiLineString',
                         'coordinates': [[[round(float(x0), 3), round(float(y0), 3)],
                                          [round(float(x1), 3), round(float(y1), 3)]]]},
        })
    return {
        'type': 'FeatureCollection',
        'name': 'warehouse_route_graph',
        'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
        'features': feats,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'warehouse_route_graph.geojson'))
    args = ap.parse_args()
    nodes, edges = build()
    with open(args.out, 'w') as f:
        json.dump(to_geojson(nodes, edges), f, indent=1)
        f.write('\n')
    print(f'wrote {os.path.abspath(args.out)}: {len(nodes)} nodes, {len(edges)} edges')


if __name__ == '__main__':
    main()
