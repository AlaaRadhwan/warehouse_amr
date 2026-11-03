# Rotation Shim — Phase 2: terminal-only shim (2026-10-26)

Branch `feature/rotation-shim-integration`. **Nothing committed or pushed.** Continues `Rotation_Shim_Integration_Phase1.md` (kept unchanged except a correction note).
Evidence classes: **[observed]** measured in these runs · **[source]** read in Nav2 1.3.12 source at the tag matching the installed packages
(only binaries/headers are installed; the plugin binary was not disassembled) · **[inferred]** · **[open]**.

## 1. Result in one paragraph

Setting `angular_dist_threshold: 3.2` makes the shim terminal-only. In 9 goal executions (2 sessions) there were **0 pre-terminal shim segments**
(Phase 1: 2 per B->A run, 183-184 deg at start plus 59 deg and 174 deg mid-route), MPPI reversed/turned normally, the shim took over inside the
goal XY tolerance, and every goal returned `SUCCEEDED`. Cost: B->A is not faster than the MPPI-only baseline (56.5-58.4 s vs baseline 39.7-57.4 s, n = 2),
terminal XY accuracy is limited by the tolerance (0.23-0.27 m at 0.25 m; 0.15 m at 0.15 m), and the shim's rotation collision look-ahead is
effectively switched off (section 5). A source finding corrects two Phase 1 concerns (section 4).

## 2. Task 1 — does `angular_dist_threshold > pi` bypass the alignment logic? **Yes, by construction and in the runs**

* **[source]** `computeVelocityCommands`: the pre-path branch runs only if `path_updated_`; inside it `angular_distance_to_heading` is
  `angles::shortest_angular_distance(...)` (range [-pi, pi]) or `atan2(y, x)` (range [-pi, pi]) and rotation starts only if
  `abs(angular_distance_to_heading) > angular_thresh`. With 3.2 > pi that is never true, so the code takes the `else` branch: `path_updated_ = false`
  and the primary controller (MPPI) runs. `in_rotation_` is therefore never set. (The unqualified `abs` could be the integer overload; Phase 1 data show
  it behaves as double - rotation started at 0.915 rad - and with 3.2 the result is the same for either overload.)
* **[source]** The terminal branch (`rotate_to_goal_heading`) is independent of that threshold: it needs only `position_goal_checker_->isGoalReached(pose, sampled_pt_goal, ...)`
  where `sampled_pt_goal` = last pose of the path and the XY tolerance is read from the goal checker (`getTolerances`).
* **[observed]** Temporary config (`make_params.py`: repo file with that one line changed; diff saved in `runs/s04_thr3p2/nav2_params.run.yaml/.diff`), runtime dump confirms 3.2. 
  `s04`: B->A shim samples 16 of 2565 (all after entering tolerance); no shim segment at distance > tolerance in any goal.
* Inert parameters in this mode: `angular_disengage_threshold`, `forward_sampling_distance`, `use_path_orientations`, `rotate_to_heading_once`. Kept in the YAML, commented.
* Side effects: see section 5 (collision look-ahead). No Nav2 source was modified.

## 3. Task 2 — functional comparison (same B-start, same goal sequence)

GT = Gazebo ground truth when Nav2 reported success. "Pre-term." = shim segments that start more than tolerance + 6 cm from the goal.

| Goal | Phase 1 s02 / s03 | Terminal-only s04 (xy 0.25) | Terminal-only s05 (xy 0.15) |
|---|---|---|---|
| **B->A time** | 42.2 / 47.9 s | 58.4 s | 56.5 s |
| B->A GT dist / yaw | 0.212 m, -0.5 deg / 0.181 m, -4.4 deg | 0.234 m, -10.7 deg | 0.154 m, +0.5 deg |
| B->A pre-term. shim | **2 (183 deg, 59 deg) / 2 (184 deg, 174 deg)** | **0** | **0** |
| C reverse 1 m: time, GT dist / yaw | 5.2 s 0.221 m +1.7 deg / 4.3 s 0.189 m -0.3 deg | 4.2 s 0.265 m +0.7 deg | 4.9 s 0.120 m +0.5 deg |
| A1 fwd + 90 deg: time, GT dist / yaw | 15.7 s 0.212 m +11.0 deg / 10.0 s 0.189 m -1.1 deg | 4.2 s 0.188 m +0.6 deg | 6.7 s 0.081 m +0.3 deg |
| A2 180 deg in place: time, GT dist / yaw | 4.8 s 0.212 m 0.0 deg / 5.1 s 0.189 m +0.2 deg | 5.1 s 0.188 m -0.0 deg | 5.2 s 0.081 m +0.1 deg |
| N1 non-lattice yaw (0.232 rad) | not run | 3.2 s, 0.188 m, **-13.6 deg vs requested** | not run |

MPPI-only baseline (9 B->A successes): 39.7-57.4 s (median 47.4), GT dist 0.017-0.135 m, yaw -11.3..+8.5 deg.

* **B->A [observed]:** the unwanted start-up and mid-route rotations are gone; the robot starts moving at once and drives the plan's own route (two cusps at x = 4.6 m and 1.7 m,
  slow section near the second cusp, smooth final approach, no dwell or oscillation near the goal). Phase 1's different route was a consequence of the 180 deg start-up rotation changing what the planner replanned.
  Timing: terminal-only 58.4 s and 56.5 s sit at the slow end of the MPPI-only spread (baseline max 57.4 s). With n = 2 this is not distinguishable from the baseline; the shim is not active before the last 0.3 s, so no shim-related cause is expected. **[inferred]**
* **Reverse (C) [observed]:** MPPI backs in (first plan: 1.2 m reverse, 0 cusps), no rotation before the terminal phase; shim terminal segment 0.2-0.8 s, <= 8 deg.
* **Final heading (A2, A1) [observed]:** the shim takes control inside the tolerance and rotates at <= 0.80 rad/s to the plan's final yaw; Nav2 succeeds; GT yaw error 0.0-0.6 deg.
  In B->A (s04) the robot arrived 8 deg off and the shim stopped as soon as the checker was satisfied (GT -10.7 deg): the shim does not polish beyond the tolerance.
* **Hand-offs at the tolerance boundary [observed]:** a few MPPI<->shim flips at the edge (s04 A1: 2 MPPI->shim, 1 shim->MPPI; s05 B->A: shim, 0.07 s MPPI, shim). No sustained oscillation, no leaving the tolerance while rotating.
* Not re-measured in Phase 2: rest-after-success profile (Phase 1: at rest within 0.5 s, no overshoot).

## 4. Correction to Phase 1 concerns (source finding)

`ControllerServer::setPlannerPath` stores `end_pose_ = path.poses.back()` and `isGoalReached()` compares the pose with `end_pose_` **[source, controller_server.cpp:635/801-818]**.
So Nav2's goal checker and the shim both judge against the **planner's last pose**, not the requested goal.

* Phase 1 item 6 (dead band between shim hand-over and goal check): **does not exist** - both use the same pose.
* Phase 1 item 5 (non-lattice yaw may fail to converge): **withdrawn**, see section 7; the actual hazard is silent heading error.
* The jitter of the end pose between replans (up to ~4.5 cm observed) moves the success target itself.

## 5. Collision safety during rotation — what is verified, what is not

* **[source, not executed]** `isCollisionFree` loops until `simulate_ahead_time` but breaks at once when `shortest_angular_distance(yaw, initial_yaw) >= fabs(d) - angular_dist_threshold`.
  With threshold 3.2 the right side is negative for every possible `d`, and the left side is `-w*t` (w > 0) or `+|w|*t` (w < 0), both of which satisfy the condition on the first iteration
  for the rates used. **Conclusion [inferred from source]: in terminal-only mode the shim's footprint look-ahead never inspects the costmap.** (With the Phase 1 threshold 0.785 it did run for |d| > 45 deg and w > 0.)
  It also only reacts to LETHAL cells (inscribed cost 253 is below 254). Not verified by an executable test; no shim collision message appeared in any run (no obstacle was in a sweep area either).
* Footprint is 1.0 x 0.74 m: in-place rotation sweeps a circle of radius sqrt(0.5^2 + 0.37^2) = 0.62 m (inscribed radius 0.37 m). Static map clearance at Station A staging is ~0.75 m; unmapped objects are unknown.
* The only active protection during shim rotation is the Collision Monitor `FootprintApproach` (1.2 s look-ahead, scan source). I did not verify that it accounts for angular velocity in this configuration, and it never reported a state in these runs (`/collision_monitor_state`: 0 messages, s04). It is not an independent safety layer.
* **Unverified:** rotation toward an obstacle, dynamic obstacles, narrow aisle terminal turn, cluttered goal, stopping distance with the 1.0 rad/s^2 profile + smoother lag, behaviour of the installed binary vs the tag source.

## 6. Task 3 — staging accuracy at Station A (separate change, `xy_goal_tolerance` 0.25 -> 0.15, temporary file, terminal-only on)

* **Geometry [source: files]:** final dock pose x = 0.799; staging pose = final - 0.70 m = (0.099, -0.175, yaw 0); AprilTag on the floor at x = 1.549 (1.45 m ahead of the staging pose). Docking: `docking_threshold` 0.05 m at the dock,
  precision controller 0.05-0.15 m/s, `initial_perception_timeout` 5 s, `dock_prestaging_tolerance` **0.50 m** (if the robot is already within 0.50 m of staging, the docking server skips the staging navigation, so then the goal tolerance is irrelevant).
  Camera: 0.465 m ahead of base, 0.24 m above floor, 25 deg down, HFOV 45.4 deg, VFOV about 35 deg (640x480).
* **Tag visibility estimate [inferred, first-order, docking not run]:** depth of the tag along the optical axis about 1.0 m, so the FOV half-width there is 0.42 m; keeping the 0.18 m tag fully inside needs the tag centre within about 0.33 m of the axis.
  Lateral tag offset ~ robot lateral error + 1.45 m * sin(yaw error): a 0.20 rad yaw error alone shifts it 0.29 m; 0.25 m XY + 0.20 rad (same sign) = 0.54 m is outside; 0.15 m + 0.20 rad = 0.44 m is still outside; 0.15 m + <= 0.12 rad is inside.
  **The yaw tolerance (0.20 rad), not the XY tolerance, dominates tag visibility.** Vertically the tag is within the FOV. 
* **Observed with 0.15 m [s05, 4 goals]:** all `SUCCEEDED`, GT final distance 0.154 / 0.120 / 0.081 / 0.081 m (vs 0.234 / 0.265 / 0.188 / 0.188 m at 0.25 m), GT yaw +0.5 / +0.5 / +0.3 / +0.1 deg, B->A 56.5 s (vs 58.4 s), no stall, no oscillation (terminal shim segments 0.2-0.8 s except the 5 s in-place 180 deg turn; MPPI re-approached the A1/A2 starting poses). AMCL-estimated distances at success exceeded the 0.15 m tolerance by up to ~1.6 cm (pose timing between the AMCL message and the checker's TF), so treat 0.15 as 0.15 +- 2 cm.
* **Why it is cheap to adopt but is not free:** the end-pose offset from the requested goal (planner `tolerance` 0.10 m; observed up to ~4.5 cm) adds to the tolerance, so the delivered distance to the requested staging pose is tolerance + offset (worst case 0.25 m for a 0.15 m tolerance). The tolerance is global (one `general_goal_checker`); a Station-A-specific tolerance would need a goal-checker choice per goal (executor/BT change - your decision). 
* **Assessment:** 0.15 m is a reasonable experimental value for Station A staging and did not reintroduce stalling in this sample (n = 4, one session). Whether it is *needed* is only answerable with an end-to-end docking attempt from the staged pose and a look at the 0.20 rad yaw tolerance. Not applied to the repo.

## 7. Task 4 — lattice heading discretization

* **[source]** the planner snaps the goal yaw to the nearest of its 16 headings (`getClosestAngularBin`, minimum `|shortest_angular_distance|`), and the path's last pose carries that bin heading (no override except for the same-cell case). Bins (deg): 0, 26.6, 45, 63.4, 90, 116.6, 135, 153.4, 180, ... Gap widths 0.3218 and 0.4636 rad; maximum snap error 0.2318 rad = 13.3 deg (8 mid-gap points).
* **Non-convergence: not a problem.** The goal checker and shim both use the plan's last pose (section 4), so the robot converges to the bin heading and Nav2 judges it there. **[observed, N1]**: requested yaw 0.2318 rad (mid-gap) -> SUCCEEDED in 3.2 s; vs the plan end pose the AMCL yaw error was -2.0 deg.
* **The real hazard is silent heading error.** N1: robot GT yaw error to the *requested* yaw -13.6 deg (AMCL estimate -15.2 deg, beyond the 11.5 deg tolerance) with `SUCCEEDED`. Worst case = up to 13.3 deg snap + 11.5 deg tolerance = about 25 deg.
  Current station yaws (0, +-pi/2, pi) and 45 deg multiples are exact bins (0 deg snap); 30 deg snaps 3.4 deg, 10 deg snaps 10 deg, 13.3 deg snaps 13.3 deg. About 8 % of the circle would exceed the 0.20 rad tolerance if the checker used the requested yaw; it does not.
* Same for XY: the checker's target is the plan end pose, up to the planner `tolerance` (0.10 m) from the request.
* **Implication for goals with arbitrary yaw:** either choose goal yaws on bins, or accept the snap, or (owner decision) add a post-check against the requested pose in the mission layer.

## 8. State of the repo and recommendation

* Applied (uncommitted): `src/amr_navigation/config/nav2_params.yaml`: `angular_dist_threshold: 3.2` + explanatory comment (the validated s04 configuration; mechanically identical in values to `runs/s04_thr3p2/nav2_params.run.yaml`); `xy_goal_tolerance` unchanged at 0.25. Phase 1 config preserved at `config/nav2_params.phase1_shim_default_thresholds.yaml`; MPPI-only baseline is Git `88a7e70`.
* **Terminal-only works, removes the unwanted rotations, and is a reasonable basis for the next stage**, subject to: (1) accepting a global XY tolerance trade-off (0.25 vs 0.15) after a docking check, (2) the disabled rotation collision look-ahead (section 5), (3) the silent snap for non-bin yaws. 
* The shim *can* deliver terminal-only behaviour through configuration; no custom controller is needed. If the look-ahead collision check must be kept, the smallest maintainable route is not a threshold change but an upstream-style fix or a thin patch of the shim's `isCollisionFree` condition - **not recommended now**, listed only as the option if rotation clearance becomes a requirement.
* **Not addressed:** the calibrated mid-route stall 0.54 m from the goal (not reproduced; unrelated to terminal logic); uncalibrated plant; statistical reliability (n = 2-4 per goal).

## 9. Runs and reproduction

`runs/s04_thr3p2` (threshold 3.2, xy 0.25, goals B->A, C, A1, A2, N1), `runs/s05_thr3p2_xy0p15` (threshold 3.2, xy 0.15, goals B->A, C, A1, A2). Phase 1: `s02`, `s03`; `s01_INVALID_driver_wait_hang` kept. Bags are local (ignored by Git).

```bash
cd investigations/rotation_shim_integration_2026-10-26
OVERRIDES="FollowPath:|angular_dist_threshold|3.2;general_goal_checker:|xy_goal_tolerance|0.15" scripts/run_shim_checks.sh RUN_ID full   # or p2 (adds the non-bin goal)
python3 scripts/analyze_shim.py runs/RUN_ID      # -> analysis/RUN_ID/summary.txt + figures
```
`scripts/make_params.py` writes a per-run copy of the params file (anchored one-line overrides) and its diff; `src/` is never touched by the harness.
