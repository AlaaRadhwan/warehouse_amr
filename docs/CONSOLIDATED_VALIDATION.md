# Consolidated navigation validation — NavFn + 2D MPPI + PreferForwardCritic + Route Server (simulation)

Branch `experiment/navfn-mppi-evaluation`. Nav2/MPPI/Route Server **1.3.12**. Labels: **[observed]** GT/log measurement, **[source]**, **[inferred]**, **[open]**. One run per test (n=1): functional evidence only — not statistical reliability, not hardware safety.

## 1. Configuration under test
`ros2 launch amr_bringup simulation_navigation.launch.py planner:=navfn structured_routing:=true mppi_path_interpretation:=2d_prefer_forward`
NavFn; MPPI `vx_min -0.35`, `use_path_orientations false`, `PathAngle.mode 0`, `enforce_path_inversion false`, `PreferForwardCritic {power 1, weight 5.0, threshold 0.5}`; goal checker `StoppedGoalChecker` xy 0.25 m / yaw 0.20 rad (unchanged); costmaps, footprint (1.0 × 0.74), calibration, other critics unchanged. Previous profiles (`lattice`, `2d`, `2d_forward_only`, …) remain selectable. Route graph unchanged (temporary).
One simulator launch (attempt 1 OK); lifecycle states and `/navigate_to_pose` + `/compute_route` servers verified before the first goal. Setup moves (S1–S4) position the robot and are not tests.

## 2. Code correction — routed goal yaw (Test 4)
**Cause [source + observed]**: the route BT skipped the "last mile" planner when the goal lay within 0.1 m of the route end (`ArePosesNear` compares position only). The Route Server path ends with the last edge's heading (0° at B), the controller's goal pose and `StoppedGoalChecker` use the path's last pose, so a requested yaw of 90° was replaced by 0° and Nav2 reported SUCCEEDED at the wrong yaw (previously: ends −90.1° off).
**Fix** (`behavior_trees/navigate_on_warehouse_route.xml`, one block): the last mile is now always planned when the goal is within 1.5 m of the route end (including the same position), with `use_start="true"` and the route end pose as start, then concatenated. The planner's path ends with the requested pose (position and yaw). The route body is unchanged, the 1.5 m guard still fails the tree rather than allowing a free-space shortcut, and no tolerance/critic value changed.
**Verified [observed]**: T4 — last-mile `/plan` (2 poses) ends at yaw 90.0°; see §3. **Not verified [open]**: a run where the yaw is deliberately unreachable to demonstrate failure; by construction the checker now compares against the requested pose.

## 3. Results (GT, simulation time)
| | T1 routed B→A staging | T2 free B→Home | T3 rear obstacle (back-out) | T4 routed Home→B, yaw 90° |
|---|---|---|---|---|
| Action result / error | SUCCEEDED / 0 | SUCCEEDED / 0 | SUCCEEDED / 0 | SUCCEEDED / 0 |
| Duration | 48.4 s (ref 49.0) | 48.2 s (ref 46.3) | 24.3 s | 39.8 s |
| Final position err | 0.066 m | 0.067 m | 0.107 m | 0.041 m |
| Final yaw err vs **requested** | −0.6° | −2.5° | −1.1° (req −90°) | **−3.5° (req +90°)** |
| Forward / reverse | 17.34 / 0.24 m | 16.05 / 0.71 m | 3.61 / 0.34 m | 16.91 / 0.13 m |
| Reverse segments (max continuous) | 1 (0.24 m) | 1 (0.71 m) | 1 (0.34 m) | 1 (0.13 m) |
| Initial heading correction | 143° in 5.9 s (to 0.3 m) | 27° in 3.3 s, then rest of turn forward | 4° | 28° |
| In-place rotation (total rot) | 49° (404°) | 177° (364°) | 101° (363°) | 27° (197°) |
| Max/mean route deviation | 0.14 / 0.05 m (ref 0.15) | – | – | 0.16 / 0.05 m |
| Min footprint clearance | 0.56 m (static map) | 0.20 m (static map, Home pocket) | 0.16 m reversing phase, **0.05 m** at goal (box) | 0.35 m (static map) |
No reverse segment exceeded the 1 m review threshold. No stall, loop or oscillation seen. Clearance = GT footprint (1.0×0.74) boundary vs the static map (5 cm raster) plus the box (exact geometry); objects absent from the map (e.g. the person model) are **not** included.
Figures: `docs/figures/val_T1_routed_B_to_Astaging.png`, `val_T2_free_B_to_Home.png`, `val_T3_rear_obstacle.png`, `val_T4_routed_Home_to_B_yaw90.png` (blue forward, red reverse, green in place).

## 4. Requirement assessment
| Requirement | Verdict |
|---|---|
| T1 matches reference; reverse short; route held; goal pose/yaw correct | **Pass** |
| T2 NavFn actually invoked (76 `/plan` messages, 661 poses ending at the goal) ; forward-preferring; reverse ≤1 m; final heading converged | **Pass** (0.71 m single reverse arc at departure) |
| T3 obstacle seen by sensors | **Pass**: LiDAR rear min range 1.508 m = expected (0.66 m rear gap + 0.85 m lidar-to-rear); local costmap marks the box region lethal (max 100, centre 99) from the first reading |
| T3 collision-free behaviour | **Pass**: no footprint–box intersection (min 0.05 m). MPPI backed only 0.34 m of the 0.66 m gap while turning (0.16 m clearance), then went around the box on the open side; it did not attempt a through-box reversal. Collision monitor `FootprintApproach` flickered to APPROACH 3 times for ≤0.13 s each (all within 0.35 s, ~22 s into the run, at the goal near the box); `cmd_vel_nav` and the monitor's output `cmd_vel` were identical at 10 Hz sampling, so no intervention was measurable **[open: sampling too coarse to claim the monitor never altered velocity]** |
| T3 no forced collision | **Pass**, but the 0.05 m terminal margin is poor (see §5) |
| T4 requested pose respected, yaw within 0.20 rad, xy within 0.25 m, stopped | **Pass** (−3.5° = 0.061 rad; 0.041 m; action SUCCEEDED only after the stopped-goal check, against the corrected path end pose). Turning clearance at B: 1.40 m > 0.622 m circumscribed radius (map) |
| Failed yaw cannot be reported as success | **By construction only** (not demonstrated) |

## 5. Limitations / open
* **T3 terminal closeness [observed]**: the goal (−7.42, 3.4) I chose is 0.8 m from the box's north face; the footprint rotated to yaw −90° within 0.05 m of the box. This is a goal-placement artefact (the goal checker and costmap inflation did not prevent it), not a rear-reverse issue, but it shows MPPI will accept a 5 cm margin at a goal near a box; physical margin policy is the owner's decision. A dedicated unsafe-reverse case (box right behind, no detour) was **not** run.
* T3 is one geometry; the 1.64 m back-out (earlier evidence) was not repeated. Low obstacles outside the LiDAR plane are **not** covered by any of these tests.
* Single runs; no repeatability; simulation only; calibration is simulator-specific. Reverse arcs at departure (0.13–0.71 m) remain by design of the soft critic.
* Startup: the first launch attempt of an earlier orchestration run hung on a lost lifecycle response (known issue; relaunched). That orchestration run was also invalid for **my own script error** (an unexecutable helper script, so no goal was ever sent; no data, no motion) — discarded, not a controller result. The final sequence had 0 unacknowledged goals. Budget: 4 primary runs + 0 repeats.

## 6. Verdict
**Accepted as the working simulation development baseline** (functional acceptance only). Not deployment-ready: no statistics, no hardware, no low-obstacle, docking, or real-station geometry validation; the route graph is not approved.
Reproduce: `investigations/consolidated_validation_2026-10-29/scripts/run_validation.sh` then `scripts/summarize.py`; per-run evidence in `runs/` (GT CSVs excluded).

## 7. Recommended next milestone
Owner-designed warehouse routes → station-aware navigation (route to staging with the requested yaw, now correct) → docking/undocking (needs a defined exit edge; reversing/undock policy) → full mission execution. Reopen controller work only on a concrete failure (e.g. a margin requirement at goals near obstacles).
