# NavFn + MPPI evaluation — **EXPERIMENTAL** (branch `experiment/navfn-mppi-evaluation`)

Not adopted. Baseline (State Lattice + MPPI path-orientation settings) is the default and is unchanged. Simulation only; Nav2 / ROS 2 Jazzy 1.3.12;
single runs (n = 1–2 per condition) — **no reliability claim**. Rotation Shim is not involved. The operational route graph is unchanged (temporary test fixture).

## 1. Technical explanation of compatibility

Installed: Nav2 / `nav2_navfn_planner` and `nav2_mppi_controller` **1.3.12** (Jazzy). Sources read: NavFn `navfn_planner.cpp` (GitHub tag 1.3.12), MPPI 1.3.12 copies already archived in
`investigations/btoa_variability_2026-10-23/analysis/addendum/mppi_1.3.12_source_used/`, Route Server sources (`goal_intent_extractor.cpp`, README).

## Verified source facts
1. **NavFn path poses carry yaw 0.** `getPlanFromPotential` writes every pose with identity orientation. `smoothApproachToGoal` replaces/appends the **last** pose with the requested goal pose (so the last pose carries the goal yaw). With `use_final_approach_orientation: true` the last yaw becomes the approach direction instead. Corner case start == goal XY: a single pose with the goal orientation.
2. NavFn assumes a **circular** robot; the footprint is not used when expanding the potential field (only inflated costmap values). Our footprint is 1.0 × 0.74 m (circumscribed 0.62 m, inscribed 0.37 m). Dijkstra is the default (`use_astar: false`); A* is optional. Plans are 8-connected-grid-derived gradient paths, not kinematically feasible.
3. **MPPI reads yaw straight from the path poses** (`utils.hpp:197`, `tf2::getYaw(pose.orientation)`); it never infers heading from path geometry.
4. `PathAlignCritic` with `use_path_orientations: true` adds `dyaw²` (trajectory yaw − path pose yaw) into the per-sample distance along the whole look-ahead. On a NavFn path (yaw 0 everywhere) this pulls the robot toward heading **+x (east)** along any path.
5. `PathAngleCritic` modes: 0 forward preference (cost = |end yaw − bearing to the look-ahead point|), 1 no directional preference (either heading or heading+π), 2 feasible orientations (uses the **path pose yaw** to decide whether the bearing is flipped by π). With NavFn yaw = 0, mode 2 flips the bearing whenever the path bearing is > 90° from east, i.e. it **prefers driving backwards for every westward path** — the opposite of what is wanted. If `vx_min` = 0 (reversing disallowed) mode 1 is silently converted to mode 0 (constructor).
6. `enforce_path_inversion: true` crops the path at the first point where consecutive segments have a negative dot product (a cusp) and tracks only up to it until the robot is within `inversion_xy/yaw_tolerance`. NavFn paths and Route Server edge paths contain no intentional cusps; a first-mile + route concatenation can contain an accidental one.
7. `GoalAngleCritic` is active only within `threshold_to_consider` (0.5 m) of `data.goal` (= last path pose) and scores trajectory yaw against the **last path pose's yaw**. `PathAlign`/`PathAngle` switch off inside the same 0.5 m radius. The controller's goal check (StoppedGoalChecker 0.25 m / 0.20 rad / stopped) is evaluated against the planner's last pose (`end_pose_`, found earlier in the Rotation Shim work) — for NavFn that equals the requested goal pose, for Route Server paths it is the last graph edge's direction, not the requested yaw.
8. `vx_min: −0.35` ⇒ `reversing_allowed_`; no critic forbids reversing, they only price it. Mode 0 prices reversing at ≈ π · weight on the end-of-horizon yaw term, it does not remove reverse samples from the sampling distribution.
9. Route Server dense paths: pose yaw = direction of the edge, so a westward edge has yaw 180° [observed first pose 180° at B departure]; the last yaw = last edge direction [observed]. These orientations are consistent with **forward** travel only.

## Which current MPPI settings are inappropriate for which path source
| Setting (baseline) | State Lattice path | Route Server path | NavFn path |
|---|---|---|---|
| `PathAlign.use_path_orientations: true` | appropriate (yaw = vehicle heading incl. reverse) | consistent only for forward travel; penalizes the 180° departure (robot faces opposite) | **wrong**: yaw 0 everywhere |
| `PathAngle.mode: 2` | appropriate | OK for forward travel | **wrong**: flips bearing → prefers reversing westward |
| `enforce_path_inversion: true` | appropriate (intentional cusps) | harmless unless a cusp is concatenated | harmless but unneeded |

## Hypotheses (not verified by source)
* H1: the 180° departure drift is caused by the mismatch between a travel-oriented path yaw and a robot that finds reversing cheaper than rotating inside the 2.8 s horizon. Test B tests whether removing the path-yaw terms (`use_path_orientations: false`, mode 0) changes it.
* H2: mode 0 + `vx_min: −0.35` still allows commanded reverse; whether forward-only (`vx_min: 0`) is needed is decided by the commands observed in tests.

## 2. Exact changes (all selectable at launch; defaults reproduce the baseline)
* `ros2 launch amr_bringup simulation_navigation.launch.py planner:=lattice|navfn mppi_path_interpretation:=baseline|2d|2d_forward_only structured_routing:=true|false`
  (`simulation_navigation.launch.py` now deep-merges overlay files over `nav2_params.yaml` into a temp file only when a non-default profile is chosen; it prints the profile.)
* `config/profiles/planner_navfn.yaml`: `GridBased.plugin: nav2_navfn_planner::NavfnPlanner`, `tolerance 0.10`, `use_astar false` (Dijkstra), `allow_unknown true`, `use_final_approach_orientation false`.
* `config/profiles/mppi_2d_paths.yaml`: `enforce_path_inversion false`, `PathAlignCritic.use_path_orientations false`, `PathAngleCritic.mode 0`. **No weights changed.**
* `config/profiles/mppi_forward_only.yaml` (adds to `2d`): `vx_min 0.0`.
* `package.xml`: `exec_depend nav2_navfn_planner`.
* **Fix in the shared route BT** (also affects the baseline, found by this evaluation): the first-mile guard was 1.5 m measured to the *pruned* route start, which rejected a robot standing 1.05 m from a node (Route Server drops a start node the robot has passed; start was 1.8 m away). Now 2.5 m; last mile unchanged (1.5 m). Note `bt_navigator` caches a BT by file name: restart Nav2 after editing the XML.
* Unchanged: dynamic limits (`vx_max 0.8`, `vx_min −0.35` except the forward-only profile), footprint, goal tolerances (0.25 m / 0.20 rad, stopped), calibration (0.92754), costmaps, localization.

## 3. Results (Gazebo ground truth; metrics from `scripts/analyze_goal.py`)
| run | dur s | travelled m | final pos err m | final yaw err ° | GT reverse m | in-place rot ° | max dev from route m |
|---|---|---|---|---|---|---|---|
| lattice+baseline MPPI, free Home→B | 35.3 | 16.53 | 0.054 | 1.0 | 0.0 | 0 | – |
| lattice+baseline MPPI, free B→A staging | 48.5 | 9.72 | 0.105 | -2.6 | 1.45 | 102 | – |
| lattice+baseline MPPI, routed Home→B | 34.5 | 16.77 | 0.041 | 1.1 | 0.0 | 0 | 0.16 |
| lattice+baseline MPPI, routed B→A staging (run 1) | 79.7 | 18.99 | 0.045 | -0.3 | 9.14 | 30 | 1.71 |
| lattice+baseline MPPI, routed B→A staging (run 2) | 81.1 | 19.17 | 0.083 | 0.1 | 9.24 | 26 | 1.71 |
| NavFn+2D MPPI, A1_free_Home_to_B | 34.7 | 16.54 | 0.062 | 0.3 | 0.0 | 0 | – |
| NavFn+2D MPPI, A2_free_B_to_Home | 54.6 | 16.56 | 0.047 | 1.8 | 16.55 | 7 | – |
| NavFn+2D MPPI, A3_free_yaw90 | 12.7 | 1.28 | 0.084 | 1.0 | 0.14 | 33 | – |
| NavFn+2D MPPI, R1_route_Home_to_B | 35.4 | 16.77 | 0.012 | 0.5 | 0.0 | 0 | 0.15 |
| NavFn+2D MPPI, R2_route_B_to_Astaging | 66.1 | 18.15 | 0.022 | -0.2 | 18.05 | 133 | 0.48 |
| lattice+2D MPPI, R1_route_Home_to_B | 35.1 | 16.82 | 0.071 | 0.2 | 0.0 | 0 | 0.16 |
| lattice+2D MPPI, R2_route_B_to_Astaging | 65.4 | 18.09 | 0.029 | 1.7 | 17.98 | 104 | 0.41 |
| NavFn+2D MPPI forward-only, A1_free_Home_to_B | 34.4 | 16.51 | 0.037 | 0.7 | 0.0 | 0 | – |
| NavFn+2D MPPI forward-only, A2_free_B_to_Home | 44.6 | 16.72 | 0.082 | 1.2 | 0.0 | 267 | – |
| NavFn+2D MPPI forward-only, A3_free_yaw90 | 12.5 | 1.12 | 0.019 | 4.3 | 0.0 | 66 | – |
| NavFn+2D MPPI forward-only, R1_route_Home_to_B | 34.6 | 15.9 | 0.017 | -0.5 | 0.0 | 0 | 0.63 |
| NavFn+2D MPPI forward-only, R2_route_B_to_Astaging | 47.7 | 17.51 | 0.066 | -0.9 | 0.0 | 120 | 0.13 |

`dur` = first motion (incl. in-place rotation) → last GT sample; "GT reverse" = distance moved opposite to the body heading; "in-place rot" = accumulated |yaw change| while speed < 0.05 m/s; deviation = distance of the GT path from the planned graph polyline.
`result.json` `sim_duration_s` is unreliable (clock subscription at goal start; e.g. 129 s for a 12 s goal) — use `metrics.json` `duration_s`.
Evidence: `investigations/navfn_mppi_evaluation_2026-10-26/runs/<run>/{metrics,result}.json`; figure `docs/figures/navfn_B_to_A_staging_comparison.png`.

### Test A — free-space NavFn + MPPI
* Home→B (no heading mismatch): fine in all profiles (0.04–0.06 m, ≤0.7°).
* **B→Home (180° initial mismatch, goal yaw = initial heading)**: with `vx_min −0.35` NavFn + 2D settings **drove the whole 16.6 m in reverse** (commands at the −0.35 m/s limit for ≈ 41 s; 54.6 s total) and arrived with correct yaw (1.8°) because it never turned. With `vx_min 0` it rotated in place (≈ 180°) and drove forward: 44.6 s, 1.2°, no reverse.
* Final orientation: yaw-90 goal converged (1.0° and 4.3°, tolerance 11.5°) in ≈ 12.5 s; no persistent spinning, no terminal stall (time within 0.5 m of goal 5–11 s in every run).
### Test B — structured-routing departure (B → A staging)
| configuration | max deviation | GT reverse | duration |
|---|---|---|---|
| baseline lattice + baseline MPPI (2 runs) | 1.71 m / 1.71 m | 9.1–9.2 m | 80–81 s |
| lattice connector + **2D MPPI** | 0.41 m | 18.0 m | 65 s |
| NavFn connector + **2D MPPI** | 0.48 m | 18.1 m | 66 s |
| NavFn + 2D MPPI + forward-only | **0.13 m** | **0 m** | 48 s |

**Separation of effects.** (i) The connector planner was *not invoked* on these runs (robot on a node; only the Route Server's single `/plan` message was published), and lattice+2D vs NavFn+2D differ by 0.07 m / 0.1 m reverse — i.e. **the improvement from 1.71 m to ≈ 0.4–0.5 m comes from the MPPI path-orientation settings, not from NavFn**. (ii) The connector itself was exercised in `navfnfwd_R1` (start 1.05 m off the HOME node; NavFn first mile): succeeded, 0.63 m max deviation (includes the free-space first mile), 0.017 m final error. (iii) 2D settings alone replace the lateral drift by a *whole-route reverse drive* (nose east, backing west, turning at corners); forward-only removes it.
### Test C — behaviors
* Persistent spinning: none. Terminal stalls: none (all goals succeeded; ≤ 11 s near goal). Orientation convergence: ≤ 4.3° in all goals. Off-route: 0.13–0.15 m (forward-only/ forward legs) up to 0.48 m (reverse drive); baseline 1.71 m.
* **Unnecessary reversing: yes — mode 0 does not prevent reverse commands**; with `vx_min −0.35` the controller reversed 16–18 m in 3 of 3 runs that started facing away from the path. A forward-only constraint (or another reverse policy) is necessary if reversing is unwanted. Reverse is slower (−0.35 vs 0.8 m/s limit) and moves with the sensors facing away from the direction of travel (consideration for the owner; not evaluated).
* Not reproduced: the earlier terminal spin/stall seen with State Lattice free-space; with n = 1–2 per condition that says nothing about frequency.

## 4. Comparison with the State Lattice + MPPI baseline
* Where baseline works (forward travel, on-axis goals) NavFn + MPPI is equivalent in these runs (Home→B 34.4–35.4 s, ≤ 0.07 m).
* Departure with 180° mismatch: baseline 1.71 m off-route / 80 s; NavFn+2D+forward-only 0.13 m / 48 s. Free-space B→Home has no lattice baseline run (the lattice free B→A run reversed 1.45 m, 102° in-place, 48 s).
* NavFn gives up: kinematically feasible paths, reverse planning/cusps (State Lattice's strength), footprint-aware search (circular assumption; our footprint is 1.0 × 0.74 m, circumscribed 0.62 m vs inflation 0.7 m — not tested in narrow gaps).

## 5. Remaining shortcomings / recommendation
* **Unverified:** narrow-aisle / footprint behavior with NavFn paths; obstacle avoidance and dynamic obstacles; repeatability (single runs); docking approach with a forward-only MPPI (DockRobot uses its own controller, so staging→dock is unaffected, but undocking/backing out needs a separate mechanism); hardware.
* `vx_min 0` forbids every reverse command from MPPI, including intentional reversing and short backing maneuvers; Nav2 `BackUp` recovery is separate. Whether the AMR should ever reverse in navigation is the owner's call.
* The 2D settings are specific to paths without heading information; they would degrade intentional-cusp lattice paths (`enforce_path_inversion false`, `use_path_orientations false`).
* Intermittent Nav2 startup hang (lost lifecycle `change_state` response; seen 2 of ≈ 9 launches in this session) — an infrastructure issue, not related to the profiles; the launch helper retries.
* **Recommendation:** NavFn + 2D-path MPPI settings **+ a forward-only (or explicit reverse-policy) decision** is **suitable for further development** as the free-space/connector option under Route Server routing — it removed the 1.71 m departure drift, with no spin/stall observed — but not for adoption: next step is a *small repeat set* (≈ 5 per scenario) of B→A staging and B→Home with forward-only, plus narrow-gap and obstacle tests, before the owner decides whether to replace State Lattice for the connectors.

## 6. Reproduction
```
colcon build --symlink-install; source install/setup.bash
bash investigations/navfn_mppi_evaluation_2026-10-26/scripts/launch_profile.sh LOGDIR structured_routing:=true planner:=navfn mppi_path_interpretation:=2d_forward_only
investigations/navfn_mppi_evaluation_2026-10-26/scripts/seq_free_and_routed.sh PREFIX     # A1,A2,A3 free + R1,R2 routed (finite timeouts, per-goal GT/command logs)
python3 investigations/navfn_mppi_evaluation_2026-10-26/scripts/recompute_all.py          # metrics from raw logs
```
