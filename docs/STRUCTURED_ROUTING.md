# Structured Warehouse Routing (Nav2 Route Server) — Milestone report

Branch `feature/structured-warehouse-routing` (from `feature/bidirectional-navigation-baseline` @ `88a7e70`).
Simulation only. Nav2 / ROS 2 Jazzy, `nav2_route` **1.3.12**. MPPI, Smac State Lattice, costmaps, footprint and the
wheel-separation calibration (0.92754) are unchanged. Rotation Shim is not part of this branch.

Labels: **[observed]** measured in simulation, **[source]** read in installed package/1.3.12 source, **[inferred]**, **[open]**.

## 1. Architecture (implemented)

```
GoToStation (amr_missions, unchanged interface)
  -> robot_executor -> NavigateToPose { behavior_tree = navigate_on_warehouse_route.xml }   (optional, off by default)
       BT: ComputeRoute (Route Server, graph edges only)
           -> first mile / last mile (Smac lattice, <= 1.5 m, else FAIL)
           -> FollowPath (MPPI, unchanged)
```

* `route_server` (stock `nav2_route::RouteServer`, already in the Nav2 lifecycle list) loads
  `src/amr_navigation/route_graph/warehouse_route_graph.geojson` (path injected at launch by `simulation_navigation.launch.py`).
* The route's dense path (0.05 m spacing) **is** the path given to the controller. Free-space planning is not used for the body of the trip, so the
  route cannot be shortcut by the planner **[observed: Home→A staging max 0.16 m off the graph polyline, vs. a much shorter free-space path]**.
* Enabled with `ros2 launch amr_bringup simulation_navigation.launch.py structured_routing:=true`. Default `false` keeps the previous
  behaviour (`navigation_behavior_tree` parameter of `robot_executor` empty → Nav2 default BT) **[observed: Home→station_b with default launch succeeds]**.
* Used because Route Server architecture (1) "dense path → controller" in the Nav2 README is the only one that enforces the corridor; (2)/(3)
  (sparse route → free-space planner / NavigateThroughPoses) let the planner leave the corridor between nodes.

### Verified facts about the installed Route Server (1.3.12) [source + observed]
* Graph: GeoJSON (`GeoJsonGraphFileLoader`); nodes = `Point`, edges = `MultiLineString` with `startid/endid`; all edges directed; ids unique across nodes+edges.
* Actions: `compute_route` (route + dense path), `compute_and_track_route` (+feedback, rerouting, operations). Services: `set_route_graph`, `DynamicEdgesScorer/adjust_edges` (close/open/re-cost edges), `ReroutingService/reroute`.
* Jazzy 1.3.12 ships the BT nodes (`ComputeRoute`, `ComputeAndTrackRoute`) but **no** route behavior tree (that is Nav2 main); `navigate_on_warehouse_route.xml` is adapted from Nav2 main's tree to the nodes present in 1.3.12.
* Start and goal poses are snapped to the **nearest graph node by Euclidean distance with no distance limit** [source `goal_intent_extractor.cpp`; observed A10: a goal inside a rack snapped to node 4, 3.6 m away]. The BT therefore guards first/last mile at 1.5 m.
* The route path's final pose heading is the **last edge's direction**, not the requested goal yaw [observed: route to HOME ends at −144°]. The goal checker compares against the path's last pose, so the graph's last edge before a station must already point along the station yaw (B and A_STAGING: 0°, satisfied).
* Corner smoothing (`smooth_corners`) is disabled so the path stays on the edges.

## 2. Coordinate system and graph

Frame `map` (origin from `maps/warehouse.yaml`, 0.05 m/px). Station coordinates are read from `amr_missions/config/stations.yaml` by `route_graph/tools/make_route_graph.py`
(single source of truth); A staging = final dock pose + `staging_x_offset (−0.70)` = (0.099, −0.175), yaw 0.

| id | name | x, y | clearance |
|---|---|---|---|
| 1 | HOME (station) | −7.589, 1.200 | 0.85 m |
| 2 | LANE_W (junction) | −5.500, 2.705 | 2.90 m |
| 3 | LANE_A_JCT (junction, Station-A branch) | −3.000, 2.705 | 4.49 m |
| 4 | A_APPROACH (corner) | −3.000, −0.175 | 1.86 m |
| 5 | A_STAGING (station staging) | 0.099, −0.175 | 1.13 m |
| 6 | STATION_B | 8.716, 2.705 | 1.45 m |

Edges: 101/102 HOME↔LANE_W, 103/104 LANE_W↔LANE_A_JCT, 105/106 LANE_A_JCT↔STATION_B (two directed edges each, "two-way lane"),
**107, 108 one-way** LANE_A_JCT→A_APPROACH→A_STAGING (final approach heading +x = dock yaw). Clearance = distance to nearest non-free map cell along the edge
(unknown counts as blocked); every edge > 0.62 m (robot circumscribed radius); minimum 0.81 m near the docking notch (full table: `docs/figures/route_graph_clearance.txt`).
Figure: `docs/figures/route_graph.png`.

### Assumptions (design choices, **not** established warehouse requirements)
* The hall is mostly open floor; lanes at y = 2.705 and the Station-A branch are my choice of centerlines, picked for clearance (≥ 1.4 m except the Home and docking notches) and so the last edge into B/A_STAGING points along +x.
* Lane traffic is two-way. The Station-A spur is one-way inbound **and has no outbound edge**: leaving A_STAGING by route is structurally forbidden until an exit/undock edge is designed (owner decision).
* No restricted zones, speed limits or intersection rules were provided; none are encoded.
* Pillars at (−7.5, 0.3) and (7.5, 0.3), the person model at (1, −1) and the shelf at (0.4, −2) constrain the area around Home and A; the graph keeps away from them.

## 3. Interaction with Smac / MPPI
Route Server replaces the global planner for the trip body; Smac lattice is only the guarded first/last-mile connector. MPPI receives the route path via `FollowPath` with unchanged parameters.
`CostmapScorer` marks an edge invalid on lethal costmap cells; `DynamicEdgesScorer` closes edges externally. The BT recomputes the route only if the goal changes or `IsPathValid` fails; recovery is bounded (2 retries, costmap clearing only) and never leaves the graph.
`bt_navigator.error_code_names` now includes `compute_route_error_code` so route failures surface as Nav2 error 405 (NO_VALID_ROUTE) instead of a misleading FollowPath 103.

## 4. Functional tests (single runs; no statistics claimed). RTF ≈ 0.53–0.60.

| Test | Result |
|---|---|
| **A** graph/route computation (`route_compute_tests.py`) | Graph loads (6 nodes/8 edges). Home→B nodes 1-2-3-6 (16.79 m); B→A_STAGING 6-3-4-5 (17.69 m); Home→A_STAGING 1-2-3-4-5 (11.05 m). A_STAGING→B and A_APPROACH→JCT: 405 NO_VALID_ROUTE (one-way respected). Unknown node 99: 400 UNKNOWN. Off-graph goal snaps to nearest node (see §1). |
| **B** Home→Station B via `GoToStation` + route BT | SUCCEEDED, 34.5 sim-s. Max deviation from route polyline **0.157 m** (free-space Smac, same goal: 0.91 m). GT final error 0.041 m / 1.1°. |
| **C-1** Home→A staging (route BT) | SUCCEEDED, 33 sim-s. Max deviation **0.158 m** around the 90° corner (route 11.05 m, travelled 10.69 m; a free-space path would be ≈ 7.8 m). GT final error 0.036 m / 0.5°. |
| **C-2** B→A staging (route BT), run twice | SUCCEEDED both times (GT final 0.045 m/−0.3° and 0.083 m/≈0°) **but off-route during departure**: max deviation **1.71 m** (p95 1.66 m), travelled 19.0–19.2 m vs route 17.7 m. See limitation L1. Free-space Smac B→A: 9.7 m, max 2.12 m from the route. |
| **D** blocked/forbidden | Edge 105 closed via `adjust_edges` → `GoToStation station_b` fails in 0.1 s with `nav2_error_code 405`, robot does not move; route compute returns 405; reopening restores the route. Executing A_STAGING→B (forbidden direction): failed, robot stayed (first run reported 103; after the error-code fix, 405 is reported for blocked edges). |
| **E** backward compat | `structured_routing:=false`: `GoToStation station_b` SUCCEEDED, unchanged behaviour. |

Evidence: `investigations/structured_routing_2026-10-26/` (run JSON/metrics/trajectory; raw GT CSVs and launch logs stay local, see its README).

## 5. Station A: staging vs docking (future)
Station A is a docking station: `GoToStation station_a` still goes through `DockRobot` with `navigate_to_staging_pose: true`, i.e. the Docking Server's own internal `NavigateToPose` with the default BT (free-space). **Unchanged in this milestone.** Planned (not implemented): executor sends the route BT to A_STAGING first; once inside `dock_prestaging_tolerance (0.50)` the Docking Server skips its own navigation; then `DockRobot` with final approach. The graph already terminates at the staging pose, not the final dock pose. Docking itself was not run.

## 6. Limitations / open issues
* **L1 — 180° departure drifts off the lane [observed, reproduced 2×].** From B (nose +x) the first route edge points −x. MPPI backs out with the nose still +x (vx ≈ −0.2…−0.26 m/s) and drifts south to ≈ 1.7 m off the lane before turning forward onto the route. It arrives correctly but violates corridor adherence for ~9 m. **[inferred]** route-path poses are oriented along travel (west) while the robot reverses with nose east, an orientation convention MPPI's reverse handling is not given by Route Server paths (Smac lattice reverse paths carry vehicle heading). No parameter was tuned. Smallest candidates (owner decision): (a) re-evaluate Rotation Shim — the route path is static, so the Phase-1 replanning-triggered rotations should not occur; (b) a small BT/path step that reverses pose headings on edges flagged as reverse; (c) a dedicated heading-aligned departure edge/behavior.
* No route-level tracking (`ComputeAndTrackRoute`), operations or speed limits are used yet; rerouting happens only through the BT goal/validity logic.
* Obstacle in a lane: edges are only closed when lethal in the global costmap; a single-lane graph has no alternative, so the mission fails (405) instead of waiting; policy for "temporarily blocked" vs "structurally forbidden" (wait/retry vs fail) is not designed. Dynamic obstacle response during `FollowPath` is MPPI + Collision Monitor, unchanged and not re-tested.
* The graph is built from my reading of the saved map; no lane-width/regulation requirements were given. Exit from A_STAGING is intentionally absent.
* Single runs only; no repeatability or hardware claims. Docking end-to-end not tested.

## 7. Reproduction
```
cd <repo>; colcon build --symlink-install; source install/setup.bash
python3 src/amr_navigation/route_graph/tools/make_route_graph.py          # regenerate graph from stations.yaml
python3 src/amr_navigation/route_graph/tools/plot_route_graph.py --out docs/figures/route_graph.png
ros2 launch amr_bringup simulation_navigation.launch.py structured_routing:=true
python3 investigations/structured_routing_2026-10-26/scripts/route_compute_tests.py out.json         # Test A
python3 investigations/structured_routing_2026-10-26/scripts/mission_run.py out.json station_b:240   # Test B
python3 investigations/structured_routing_2026-10-26/scripts/mission_run.py out.json "nav@0.099@-0.175@0.0@bt:240"  # Tests C
ros2 service call /route_server/DynamicEdgesScorer/adjust_edges nav2_msgs/srv/DynamicEdges "{closed_edges: [105]}"  # Test D
```
Note: `amr_navigation`/`amr_description` install empty `rviz/`/`meshes/` directories; `.gitkeep` files were added so a fresh clone builds.
