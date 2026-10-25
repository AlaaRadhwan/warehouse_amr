# Navigation test watchdog (test harness only)

Files in `calibration/nav_validation/`: `nav_watchdog.py` (rules, no ROS), `nav_watchdog.yaml` (thresholds), `nav_goal.py` (live client, now uses it), `replay_watchdog.py` (offline replay over recorded bags), `test_nav_watchdog.py` (synthetic scenarios). The production controller, MPPI, progress checker and goal checker are not read or modified.

## Why the old watchdog failed
It armed only within 1 m of the goal, so the uncalibrated control (spin locked in 14.8 s after the goal, 7.7 m out, 3491 deg) was never seen. It also used raw `wz > 0.5` for 12 s, which would also fire on a legitimate slow heading correction.

## Design
Each rule is "rotating a lot" AND "not getting closer", so a bounded in-place correction followed by progress or goal success never trips, but a spin that goes nowhere does. Progress is NET approach (1 s averages at both window ends) of the larger of Nav2 `distance_remaining` and the straight-line distance, not best-case, because a spinning robot jitters and drifts slowly (a best-case metric credited the control with progress and caught it only at the end of the recording).

| rule | fires when | default | justification |
|---|---|---|---|
| saturated_spin | >=80% of 8 s has `abs(wz)` >= 0.9 x `wz_max` (0.9 rad/s) with one sign AND < 2 m progress per turn | sat_window 8 s, 0.8, 2 m/turn | legit runs peaked at 0.43 rad/s, never above 0.5; a pi turn at the 1.0 limit takes 3.2 s, 8 s is >2x |
| rotation_no_progress | >=1 turn of accumulated yaw in 20 s AND < 2 m/turn | 1.0 turn, 20 s | worst legit rotation in any 20 s window 0.77 turn (171 deg correction + path curvature) |
| dithering | >=6 left/right `wz` reversals (|wz|>0.3) in 20 s AND < 0.3 m progress | 6 | legit runs: 0-1 reversals |
| stalled | < 0.3 m net progress in 45 s while farther than the goal xy tolerance | 45 s | longest legit no-progress phase (Nav2 spin/backup recovery) is < 15 s |
| settle_timeout | inside 0.25 m of goal for 30 s without success | 30 s | calibrated runs settled in <= 15 s; this is the in-tolerance-but-never-stops case (yaw or velocity not converging) |
| recoveries | Nav2 reports >= 4 recovery attempts | 4 | |
| timeout | goal older than 240 s | 240 s | |
A 3 s grace after goal acceptance is ignored. All values are in `nav_watchdog.yaml`; override per run with `nav_goal.py --config FILE` or `--set sat_window=6 rot_max_turns=1.5`. Keep `wz_max` equal to the MPPI value, and `goal_xy_tol` equal to the StoppedGoalChecker value, by hand (the harness does not read Nav2 params).

Legitimate vs pathological: a 180 deg in-place correction is 0.5 turn, so it trips none of the turn-based rules; two back-to-back pi turns at the same spot with no progress (1 turn, 0 m/turn) does trip, intentionally. A saturated turn lasting < 6.4 s (80% of 8 s) does not trip. Slow continuous spinning (0.5 rad/s, below saturation) is caught by rotation_no_progress after 20 s; constant motion toward the goal while turning is not.

## Validation (no live simulation was run for this)
Offline replay over the recorded bags (`replay_watchdog.py RUN...`, uses `/cmd_vel` and the action feedback):

| run | result | peak turns in 20 s | peak saturated fraction | max abs wz |
|---|---|---|---|---|
| calibrated #1, #2, #3 | no trigger | 0.70, 0.77, 0.63 | 0.0 | 0.43 |
| invalid control (multiplier not applied) | no trigger | 0.68 | 0.0 | 0.43 |
| uncalibrated control | **rotation_no_progress at 19.7 s** after goal (1.50 turns, 1.07 m/turn); spin began 14.8 s | 1.50 | 0.62 (at trigger) | 1.00 |

So the watchdog would have stopped the far-field spin ~5 s after it locked in instead of the operator stopping it at ~69 s. `test_nav_watchdog.py` (synthetic): cruise, in-place pi turn then drive, and 171 deg goal correction do not trip; pinned spin, slow spin, dithering, stuck, settle-forever and recoveries each trip the intended rule (9/9 pass).

## Limitations
- Margin is not large on legitimate runs: peak 0.77 turn vs the 1.0 limit. Only 4 legitimate runs of one goal geometry were used; other missions (docking staging, tight aisles, recovery behaviours) may need `rot_max_turns` or the m/turn limits adjusted. Replay new runs before trusting them.
- `distance_remaining` and the pose come from Nav2 feedback (AMCL, error up to 0.18 m), so the watchdog sees Nav2's belief, not Gazebo ground truth.
- Live wiring in `nav_goal.py` is syntax and argument checked and uses the same code path as replay, but has not been run against a live simulation yet.
- Uses the final `/cmd_vel` wz, not the plant rate, so it cannot detect a plant/command mismatch by itself (that is what the kinematic calibration tests are for).
