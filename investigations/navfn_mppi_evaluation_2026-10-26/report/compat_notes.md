# NavFn + MPPI compatibility notes (verified source facts vs hypotheses)

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
