# Accepted simulation navigation development baseline

Status: **accepted for continued simulation development only.** Evidence is single-run functional validation in Gazebo (RTF ≈ 0.6); no statistics, no hardware, no low-obstacle, docking or real-station geometry validation. Not deployment-ready. Simulator calibration (0.92754) is not hardware calibration; Nav2 Collision Monitor is not a substitute for independent physical safety.

## Configuration
| Component | Setting |
|---|---|
| Stack | ROS 2 Jazzy, Nav2 / Route Server 1.3.12 |
| Global planner | NavFn (`planner:=navfn`) |
| Controller | MPPI, 2D-path interpretation: `PathAlignCritic.use_path_orientations false`, `PathAngleCritic.mode 0`, `enforce_path_inversion false` (`config/profiles/mppi_2d_paths.yaml`) |
| Reverse | enabled, `vx_min -0.35`; soft forward preference `PreferForwardCritic {cost_power 1, cost_weight 5.0, threshold_to_consider 0.5}` (`config/profiles/mppi_prefer_forward.yaml`) |
| Routing | Nav2 Route Server, `structured_routing:=true`, BT `behavior_trees/navigate_on_warehouse_route.xml` (ComputeRoute → guarded first/last mile → MPPI) |
| Drive | `wheel_separation_multiplier 0.92754` (`amr_control/config/controllers.yaml`) |
| Goal checker | `StoppedGoalChecker` xy 0.25 m, yaw 0.20 rad (unchanged) |
Launch:
```bash
ros2 launch amr_bringup simulation_navigation.launch.py planner:=navfn mppi_path_interpretation:=2d_prefer_forward structured_routing:=true
```
Defaults (`planner:=lattice mppi_path_interpretation:=baseline structured_routing:=false`) reproduce the previous baseline; all earlier profiles stay selectable. Profiles are YAML overlays deep-merged over `config/nav2_params.yaml` at launch; no base parameter was edited.

## Why this combination
* NavFn paths carry yaw 0 except the last pose, so MPPI's path-orientation terms had to be switched off (`2d` settings) — with them on, the 180° departure at Station B drifted 1.71 m off the lane and reversed ~9 m (`NAVFN_MPPI_EVALUATION.md`).
* Reverse allowed without a preference reversed whole routes (≈18 m); forward-only (`vx_min 0`) removed useful reversing. `PreferForwardCritic` 5.0 cut reverse to 0.1–0.7 m per run and kept a 1.6 m back-out possible (`PREFER_FORWARD_CRITIC_EVALUATION.md`).
* State Lattice with reverse expansion matched NavFn within single-run noise; without reverse expansion it stalled near Home (`LATTICE_2D_FORWARD_EVALUATION.md`). NavFn was chosen as the simpler option, not proven better.
* Route Server dense paths keep the robot within ≈0.15 m of the lane (`STRUCTURED_ROUTING.md`).

## Behavior and limitations
* **Reverse** is a soft preference: expect short back-out arcs at departure (0.1–0.7 m observed; none >1 m). It cannot distinguish useful from convenient reversing; if station exits must guarantee/limit reversing use an explicit bounded maneuver (undock), not critic tuning.
* **Terminal yaw (corrected)**: the Route Server path ends with the last edge's heading, which previously replaced the requested yaw (Nav2 reported SUCCEEDED at the wrong yaw). The route BT now always plans the last mile (goal within 1.5 m of the route end) so the final path pose is the requested goal pose; verified with a 90° request at B (−3.5° error). A deliberately unreachable yaw was not tested.
* **Rear obstacle**: one test with a runtime-spawned box behind the robot — detected by LiDAR and local costmap, no intersection, but a 0.05 m footprint margin occurred at a goal placed close to the box. Margin policy at goals is undecided. Obstacles below the LiDAR plane are not covered.
* **Route graph is provisional**: a temporary fixture (6 nodes, 8 edges, no outbound edge from Station A staging, no restricted zones/speed limits). Structured routing is opt-in until the owner approves a graph. Docking/undocking through the route BT is not integrated (Station A still uses the Docking Server's own navigation).
* **Startup**: Nav2 startup occasionally hangs on a lost lifecycle response (~2 of 9 launches observed); verify `lifecycle_manager_navigation` and node states, relaunch if needed. The first goal after launch is sometimes not acknowledged by the test client.
* Rotation Shim work (`feature/rotation-shim-integration`) is separate and unfinished; MPPI `visualize: true` remains a diagnostic setting.

## Evidence
`docs/CONSOLIDATED_VALIDATION.md` (4 tests: routed B→A, free B→Home, rear obstacle, routed yaw 90°; scripts in `investigations/consolidated_validation_2026-10-29/`), `PREFER_FORWARD_CRITIC_EVALUATION.md`, `NAVFN_MPPI_EVALUATION.md`, `LATTICE_2D_FORWARD_EVALUATION.md`, `STRUCTURED_ROUTING.md`. Raw GT recordings are intentionally not in Git.

## Next milestone
Owner-designed warehouse routes → station-aware navigation → docking/undocking → complete mission execution. Reopen controller work only for a concrete failure.
