# Rotation Shim integration — Phase 1 (2026-10-26)

> **Update 2026-10-26 (Phase 2):** your decision was terminal-only shim; see `Rotation_Shim_Phase2_Terminal_Only.md`.
> Two items below were **corrected** by a source finding (the controller server's goal checker compares against the *planner's last path pose*, not the requested goal):
> item 5 (non-lattice yaw "non-convergence") is withdrawn - the real effect is a silent heading error; item 6 (dead band) does not exist.
> Items 1-3 are resolved by `angular_dist_threshold: 3.2` (0 pre-terminal shim segments in Phase 2). This file is otherwise unchanged as the Phase 1 record.


Branch `feature/rotation-shim-integration` (from `88a7e70`, pushed baseline). **Nothing committed or pushed.**
Scope: wrap MPPI with `nav2_rotation_shim_controller::RotationShimController`; keep Smac State Lattice, MPPI weights,
wheel calibration and goal tolerances. Short functional checks only (2 runs x 4 goals), not a statistical evaluation.

Evidence classes below: **[observed]** measured in these runs; **[source]** read in the Nav2 1.3.12 source
(`nav2_rotation_shim_controller.cpp`, `smac_planner_lattice.cpp`; installed plugin is 1.3.12, only binaries/header are installed,
so source was fetched at the matching tag); **[inferred]** reasoned from both; **[open]** not established.

## 1. Files changed

| File | Change |
|---|---|
| `src/amr_navigation/config/nav2_params.yaml` | `FollowPath.plugin` -> `nav2_rotation_shim_controller::RotationShimController`; added `primary_controller: nav2_mppi_controller::MPPIController` and 10 shim parameters (below). Mechanically verified: the only changed existing value is `FollowPath/plugin`; 11 keys added; all MPPI, critic, State Lattice, goal-checker, smoother and calibration values are identical to `HEAD`. |
| `src/amr_navigation/package.xml` | `exec_depend nav2_rotation_shim_controller` (installed: ros-jazzy-nav2-rotation-shim-controller 1.3.12). |
| `investigations/rotation_shim_integration_2026-10-26/` (new, untracked) | `scripts/run_shim_checks.sh`, `shim_client.py`, `analyze_shim.py`, `config/topics.txt`, runs `s02`, `s03` (+ `s01_INVALID_driver_wait_hang`), `analysis/`, this report. No `src/` code, no launch file and no other parameter was touched. `install/` symlinks the params file, so no rebuild was needed. |

## 2. Resulting controller configuration

```
FollowPath:
  plugin: nav2_rotation_shim_controller::RotationShimController
  primary_controller: nav2_mppi_controller::MPPIController     # MPPI parameters stay under FollowPath.* [source: shim configures the primary with the same plugin name]
  angular_dist_threshold: 0.785   angular_disengage_threshold: 0.3925   forward_sampling_distance: 0.5   (defaults, made explicit)
  use_path_orientations: true   rotate_to_heading_once: true   rotate_to_goal_heading: true   closed_loop: true   (as specified)
  rotate_to_heading_angular_vel: 0.8   max_angular_accel: 1.0   simulate_ahead_time: 1.0
```
Rationale for the two chosen values: **0.8 rad/s** = 80 % of MPPI `wz_max` (1.0), below the smoother's 1.2 max, and away from the pinned-limit
regime seen in `e01` (1.0 commanded, 1.079 measured in the uncalibrated plant). **1.0 rad/s^2** = MPPI `az_max`; below the smoother's
1.5 rad/s^2 so the shim's own stopping profile, not the smoother, shapes the rotation. Both are derived from existing limits, not tuned.

## 3. Did it load and operate? **[observed]**

* Log: `Created controller : FollowPath of type ...RotationShimController`, `Created internal controller for rotation shimming: ... MPPIController`, `Configured/Activated MPPI Controller: FollowPath`. Nav2 activated; no errors.
* Runtime `ros2 param dump` (s02): all shim values above in effect; MPPI `wz_max 1.0, az_max 1.0, vx_min -0.35, regenerate_noises false, enforce_path_inversion true`; goal checker `StoppedGoalChecker 0.25 / 0.20 / rot_stopped 0.1`; `wheel_separation_multiplier 0.92754`; planner `SmacPlannerLattice`, reverse expansion true.
* Command source per `/cmd_vel_nav` sample is **inferred** (no published signal): a sample is MPPI's iff an `/optimal_trajectory` arrived <=5 ms before it in bag time (measured offset 0.1 ms, p99 0.7 ms, run s02). The shim's rotation commands are visible as vx = 0, |wz| up to exactly 0.80.

## 4. Functional results (two identical runs, robot started at Station B)

All 8 goals returned `SUCCEEDED`. GT = Gazebo ground truth at the moment Nav2 reported success.

| Goal | run | sim s | GT dist to goal | GT yaw err | shim segments (dyaw GT) |
|---|---|---:|---:|---:|---|
| B -> A staging (0.099,-0.175,0) | s02 | 42.2 | 0.212 m | -0.5 deg | 183 deg @start; 59 deg @0.54 m; terminal 0.8 s |
| | s03 | 47.9 | 0.181 m | -4.4 deg | 184 deg @start; **174 deg @0.91 m** (5.4 s); terminal 0.2 s |
| C reverse 1 m (-0.9,-0.175,0) | s02 / s03 | 5.2 / 4.3 | 0.221 / 0.189 m | +1.7 / -0.3 deg | terminal only (8 / 2 deg) |
| A1 fwd 0.6 m + 90 deg (-0.3,-0.175,pi/2) | s02 / s03 | 15.7 / 10.0 | 0.212 / 0.189 m | +11.0 / -1.1 deg | s02: 47 deg start, 24 deg mid, terminal; s03: 40 deg start, terminal 49 deg |
| A2 in place, 180 deg (-0.3,-0.175,-pi/2) | s02 / s03 | 4.8 / 5.1 | 0.212 / 0.189 m | +0.0 / +0.2 deg | one shim segment, 169 / 179 deg |

Baseline (MPPI only, same B->A start, 9 successful runs `btoa_variability_2026-10-23`): 39.7-57.4 sim-s (median 47.4), **final GT distance 0.017-0.135 m (median 0.05)**, final yaw -11.3..+8.5 deg.

* **Test A — final orientation: works.** A2 (pure 180 deg turn, 4.8-5.1 s): shim rotates at <=0.80 rad/s with the specified acceleration profile, GT yaw error 0.0-0.2 deg. In every goal the shim takes over inside 0.25 m of the path end pose. In 7 of 8 goals MPPI is not called again afterwards; in s02 goal C there was one shim -> MPPI -> shim hand-back (MPPI for 0.5 s at 0.24 -> 0.22 m) because the replanned end pose moved. After Nav2 reports success the robot is at rest within 0.5 s (fused wz <= 0.009 rad/s, GT yaw change after success <= 0.2 deg, no overshoot). At the instant of success the angular rate is still small but not zero (average over the last 0.2 s: 0.0-0.19 rad/s fused; the checker's `rot_stopped_velocity` is 0.10), so "at rest" is reached just after success, not before.
* **Test B — normal B->A: succeeds in time comparable to the baseline** (42-48 s), terminal heading good, **but the approach differs from the baseline** (section 5, items 1-3).
* **Test C — reverse: no unnecessary 180 deg rotation for a simple reverse.** First plan reverse 1.2 m, 0 cusps; MPPI backs in; shim only acts inside tolerance. `use_path_orientations: true` does what it is documented to do for reverse segments **[observed, 2/2]**. For the cusp-containing B->A path, see item 3.

## 5. Findings that need your attention

1. **Start-up rotation chases a moving target (observed 2/2; mechanism [source]+[observed]).** B->A begins with a 5.3-6.2 s in-place rotation of 183-184 deg with vx = 0, then MPPI drives. The baseline starts moving at once (about 1.0 m in 8 s, yaw excursion 46-148 deg in 12 s). Cause: every 1 Hz replan starts from the robot's *current* heading, so the 0.5 m sample-point heading changes with each replan (52 deg -> -21 deg -> -169 deg -> -154 deg while the robot yaw rose 0 -> 169 deg), and `in_rotation_` makes each replan count as "goal changed" so the shim keeps rotating until the robot is within 22.5 deg of the *latest* target. Net cost in total time was small; the behaviour is different from the baseline and not what "rotate to the path heading once" suggests.
2. **Mid-route shim re-activation by replanning (observed 2/2; trigger [source]+[observed]).** `rotate_to_heading_once` re-arms when the *last pose* of the new path differs from the previous one (exact comparison, `isGoalChanged`). The Smac lattice end pose is not always the requested goal: planner `tolerance: 0.10 m` yields end poses up to ~4 cm away, alternating between replans (s02: 3 distinct end poses; s03: 11). The first changed end pose coincides with a shim take-over in both runs (s02 36.52 s -> shim 36.54-38.49 s, 59 deg at 0.54 m; s03 34.59 s -> shim 34.61-39.98 s, 174 deg at 0.91 m). The shim rotates because the sampled path heading differs from the robot yaw by more than 45 deg at that point, which near the last cusp is expected; but the *trigger* is the end-pose jitter, not a new goal. This is the "repeated shim activation caused by replanning" risk you named, confirmed to happen.
3. **Cusp region: whether the 174 deg rotation is "correct" is not established.** In s03 the robot (yaw -167 deg) rotated in place to +7 deg at 0.91 m from the goal, matching the path's heading after the cusp (+17..+47 deg), i.e. consistent with the plan. MPPI-only handled the cusp with path inversion without a long stationary rotation. I did not test the same cusp without the shim in this session.
4. **Terminal position accuracy is lower (observed 8/8).** The shim hands over at the goal checker's XY tolerance, so the robot stops at 0.18-0.22 m (GT) instead of the baseline's median 0.05 m. This is a direct consequence of "use the tolerance (0.25 m) as the take-over radius", which you told me not to change. If Station A staging needs better than ~0.2 m, that is a design decision.
5. **Goal yaw is the path's last-pose yaw, i.e. snapped to the nearest of 16 lattice headings [source: `getClosestAngularBin`; no override except the same-cell case].** All tested goal yaws (0, +-pi/2) are lattice bins, so no effect was seen. For other yaws the shim rotates to the bin heading; neighbouring bins are up to 0.46 rad apart, so the error can reach ~0.23 rad, more than the 0.20 rad yaw tolerance. **[open: not tested]**
6. **XY take-over is measured to the path end pose, success to the real goal.** With end poses up to ~4 cm off, a narrow band exists where the shim holds control but the goal checker is not yet satisfied. Not seen (success was declared immediately on entry), but progress-checker abort would be the only exit. **[open]**
7. **Not exercised:** rotation collision handling (`isCollisionFree`, 1.0 s look-ahead; no obstacle near the rotations, no collision messages in either run); the calibrated mid-route stall 0.54 m from the goal (not reproduced; shim take-over at that distance happens only through item 2, so it is *not* addressed by the terminal logic); the uncalibrated plant; narrow-aisle rotation; docking with a shim-delivered staging pose.

## 6. Verified / not verified

* Verified: plugin load, parameter values in effect, no change to any other parameter (mechanical diff), 8/8 goal successes, command-source discrimination (timing), terminal rest, GT final errors, baseline comparison (different runs, different day).
* Not verified: statistical behaviour (n = 2 per goal), mechanism claims in item 2 beyond the source + timing coincidence (a controlled replan-jitter test was not run), any hardware behaviour, `colcon build`, lint tests.
* Run integrity: `s01` is **invalid** (my driver script `wait` also waited on the background launch; no goals were sent; kept, labelled). `s02` and `s03` are complete; infrastructure normal.

## 7. Readiness

Integration is **functionally working and understandable**, ready for *further validation*, but I would **not** adopt it as the new baseline before you decide on items 1, 2 and 4 (they are behaviour/architecture decisions, not tuning). Candidate responses to discuss (none applied): make the shim see a stable goal pose (planner end-pose jitter), compare against the baseline cusp handling with an A/B at equal conditions, or accept the larger terminal position error.

## 8. Reproduce

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
investigations/rotation_shim_integration_2026-10-26/scripts/run_shim_checks.sh RUN_ID full   # one simulation, 4 goals, ~4 min wall
python3 investigations/rotation_shim_integration_2026-10-26/scripts/analyze_shim.py investigations/rotation_shim_integration_2026-10-26/runs/RUN_ID
```
Bags (~29 MB per run) stay local (`*.mcap` is ignored). Per-run `gt.csv`, client results, parameter dumps, `analysis/<run>/summary.txt` and figures are small.
