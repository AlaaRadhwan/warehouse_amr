# Wheel-separation calibration milestone: validation and B -> A retry

Date: 2026-10-22. Branch `feature/bidirectional-navigation-baseline`. Nothing was committed.
Nav2, MPPI, State Lattice, EKF, goal checker and URDF were not modified.

## 1. Exact configuration diff

```diff
--- a/src/amr_control/config/controllers.yaml
+++ b/src/amr_control/config/controllers.yaml
@@ -24,7 +24,7 @@ diff_drive_controller:
     wheel_separation: 0.69
     wheel_radius: 0.10
 
-    wheel_separation_multiplier: 1.0
+    wheel_separation_multiplier: 0.92754
     left_wheel_radius_multiplier: 1.0
```

`install/amr_control/.../controllers.yaml` is a symlink into `src/`, so no rebuild was needed. The running controller was checked after launch: `wheel_separation 0.69`, `wheel_separation_multiplier 0.92754`, `wheel_radius 0.1` (logged in each run's `controller_params.txt`). The other uncommitted changes in the repo (`nav2_params.yaml`, `package.xml`, `Findings/`, `debug/`) were left as they were.

## 2. Kinematic calibration, before / after

Same experiments as the baseline, one clean simulation each (`calibration/yaw_rate_sysid/runs_after/`, comparison in `reports/before_after_calibration.md`). Error = ratio - 1, mean over steps (worst step in brackets).

| quantity | before | after | target |
|---|---|---|---|
| true yaw rate / command (+-0.4, +-0.7, +-1.0 rad/s) | 1.07812 (+7.81%) | 1.00000 (0.000%) | |
| raw wheel odometry yaw rate / ground truth | 0.92759 (-7.24%) | 1.00002 (+0.002%, worst 0.007%) | < 0.5% |
| wheel velocities / ground truth | 0.92754 | 1.00000 | |
| IMU gyro / ground truth | 1.00000 | 1.00000 | |
| fused (EKF) twist yaw rate / ground truth | 0.99996 | 1.00000 | |
| **fused pose-yaw slope / ground truth** | **0.98205 (-1.80%)** | **1.00000 (0.000%, worst 0.001%)** | |
| true longitudinal velocity / command (+-0.3, +-0.6 m/s) | 1.00000 | 1.00000 | < 1% |
| raw odom longitudinal / ground truth | 1.00002 (worst 0.003%) | 1.00001 (worst 0.001%) | < 1% |
| GT lateral velocity, straight line | 9e-11 m/s | 8e-11 m/s | no drift |
| GT yaw rate, straight line | 3e-13 rad/s | 1e-15 rad/s | no drift |

All acceptance targets pass with large margin. The EKF pose-yaw discrepancy is gone: the 1.8% shortfall came from the EKF blending a wheel yaw rate that was 7.2% low with a correct IMU rate, and disappears when the wheel model is right.

## 3. B -> A navigation

Procedure (identical for every run, `calibration/nav_validation/run_b_to_a.sh`): one simulation + full Nav2 via `amr_bringup simulation_navigation.launch.py`; Home -> Station B; then NavigateToPose to the Station A staging pose (0.099, -0.175, yaw 0), the same goal as the earlier `debug/nav_test_b_to_a_bidirectional` test. Recorded: raw `/unsmoothed_plan`, smoothed `/plan`, `/transformed_global_plan`, MPPI `/optimal_trajectory`, `/cmd_vel_nav` (MPPI output), `/cmd_vel_smoothed`, `/cmd_vel` (final), controller-limited output, raw and fused odometry, IMU, `/amcl_pose`, `/tf`, action feedback/status, and Gazebo ground truth. Run 1 also has `/trajectories` (the 3.3 GB reason it is so large). A watchdog would cancel the goal on a sustained spin within 1 m of the goal, on 360 degrees of near-goal rotation, or after 240 s.
Goal checker (unchanged): StoppedGoalChecker, xy 0.25 m, yaw 0.20 rad, stopped 0.05 m/s and 0.10 rad/s, stateful false.

| run | multiplier in controller | result | sim time | max cmd \|wz\| | time within 1 m of goal | GT final xy error | GT final yaw error | GT v / wz when goal accepted |
|---|---|---|---|---|---|---|---|---|
| calibrated #1 | 0.92754 | SUCCEEDED | 47.3 s | 0.43 | 19.1 s | 0.112 m | -4.2 deg | 0.009 m/s / 0.060 rad/s |
| calibrated #2 | 0.92754 | SUCCEEDED | 56.5 s | 0.43 | 8.5 s | 0.015 m | -2.6 deg | 0.031 / 0.028 |
| calibrated #3 | 0.92754 | SUCCEEDED | 60.6 s | 0.43 | 12.0 s | 0.057 m | +0.5 deg | 0.002 / -0.055 |
| **uncalibrated control** | **1.0 (loaded at launch)** | **spin; stopped by operator at 69 s** | n/a | **1.00** (pinned) | never got within 5.0 m | n/a | n/a | n/a |

Calibrated runs, terminal behaviour:
- **Heading convergence:** run 1 arrived at the goal position facing about 175 deg away, then rotated once, smoothly, through 171 deg (yaw error -175 -> -4 deg over about 15 s, peak 0.36 rad/s) and stopped. Runs 2 and 3 approached in reverse (cmd vx -0.35 to 0) and needed only 35-77 deg of rotation near the goal.
- **Sustained angular velocity:** none. Ground-truth |wz| never exceeded 0.38 rad/s; zero seconds above 0.5 rad/s in all three runs; MPPI never approached `wz_max = 1.0` (max 0.43-0.45).
- **Forward/reverse cusps:** the first global plan has 2 cusps in all runs (about 9.6-10.0 m, 18-24% of it reversing, 8-16 rotate-in-place poses), so bidirectional planning is active. The transformed plan handed to MPPI contained no cusp in any message inside the final 1 m zone (0 of 569, 171, 273). Run 1 had 2 forward/reverse command changes near the goal, runs 2 and 3 none.
- **Settling inside StoppedGoalChecker tolerances:** yes in all three. Goal success was declared when ground truth was 0.002-0.031 m/s and 0.028-0.060 rad/s, inside the 0.05 / 0.10 stop thresholds and inside the 0.20 rad yaw tolerance (residual yaw 0.5-4 deg). Final xy error 0.015-0.112 m against a 0.25 m tolerance.
- **Nav2's belief vs truth:** Nav2 reported distance remaining 0.0 at success, but ground truth was 0.015-0.112 m from the goal. AMCL error against Gazebo was up to 0.18 m (mean 0.03-0.05 m) in the same runs. It is real and within tolerance, but it is larger than the goal checker's stop precision.
- **Yaw-rate agreement during motion:** raw odometry, fused odometry, IMU and cmd_vel all agree with ground truth within 0.2-2.1% when |wz| > 0.2 rad/s. The small positive offset (mostly +1-2% on raw odometry and cmd_vel, +0.2-1.4% on IMU/fused) is larger than the 0.007% steady-state error and is probably a timing/lag artefact of comparing against a 0.5 s finite-difference ground-truth rate during rapidly changing commands; I did not isolate it.

Uncalibrated control (multiplier 1.0 genuinely loaded from a scratch controllers file, repo untouched): a spin locked in 14.8 s after the goal was sent, at 7.7 m from the goal. MPPI's output was then pinned at wz = 1.00 rad/s for about 55 s while ground-truth yaw rate was 1.058-1.08 rad/s (above the 1.0 limit), raw odometry read 1.00, IMU and fused read 1.00 x ground truth. The robot made 3491 deg (9.7 turns) of rotation and drifted only from 7.7 m to 5.0 m from the goal in a looping, cycloid-like path (`partial_run_overview.png`). I stopped it on request. It was not tested near the goal at all.

Earlier failure for reference (same goal, pre-calibration, `debug/nav_test_b_to_a_bidirectional`, analysed in its `REPORT.md`): near the goal the robot spun at wz -1.001 pinned for more than 60 s with the IMU reading -1.079.

## 4. Findings

Confirmed:
1. The calibration works: all targets pass, including the EKF pose-yaw error.
2. With the correction, B -> A completed 3 of 3 times with smooth, bounded terminal behaviour and stopping velocities inside the checker thresholds.
3. With the original multiplier, loaded for real, the identical mission locks into a sustained ~1 rad/s spin (1 of 1 valid control; plus the earlier real failure with the same signature). In that spin the true yaw rate exceeds `wz_max` by the calibration factor, which matches the mechanism predicted by the earlier offline analysis (plant faster than the commanded rate leaves MPPI's sampled sequence always clipped at the limit).
4. Together, 3 and 2 support, but do not by themselves prove, that the kinematic error was the cause of the spin: the only intended difference between the control and the calibrated runs is the multiplier.

Unresolved / caveats:
- **Sample size.** 3 calibrated vs 1 valid uncalibrated control. MPPI is stochastic and the three calibrated runs chose different approaches (one forward + 171 deg turn, two reverse). More control and calibrated repeats would tighten the claim.
- **The control failed in a different place** (7.7 m out, during cruise) than the earlier near-goal failures. The spin signature is the same, but this run says nothing about the near-goal region; I cannot say whether the earlier near-goal lock-in would reproduce.
- **The State Lattice / MPPI path-association issues from the earlier notes were not exercised or fixed.** The calibrated runs show no cusp in the transformed plan inside the last metre, but 0 cusps near the goal in 3 runs does not show the cusp-truncation problem cannot occur (it did occur in the earlier failure).
- Only this one goal geometry was run (Home -> B -> A staging). Other goals, including yaw-pi variants, were not tested.
- AMCL localization error of up to 0.18 m versus ground truth is unexplained and not investigated.
- The 7.8% / 0.64 m effective separation is a property of this simulation's wheel contact model (see the kinematic identification report); it must be re-identified for real hardware.

## 5. Process notes and mistakes

- **Invalid first control run.** I first tried to restore the uncalibrated model by `ros2 param set wheel_separation_multiplier 1.0` at runtime. The parameter changed but the controller did not apply it (raw odometry still matched ground truth), and that run succeeded. It is kept as `runs/..._INVALID_control_runtime_param_set_not_applied` and must not be counted as a control.
- **RViz was left open** after earlier runs because my shutdown script did not list `rviz2`. Fixed in `kill_sim.sh`; the last runs and the final state show no `rviz2`, `gz` or Nav2 processes.
- **Disk is 98% full (1.2 GB free).** Run 1's bag is 3.2 GB because of `/trajectories` (about 1.9 MB per message at 20 Hz); later runs omit it (about 40 MB each). Consider moving or deleting that bag after review.
- The recorder is now given time to finalize bags; the five bags here were rebuilt with `ros2 bag reindex -s mcap`.
- The watchdog only arms within 1 m of the goal, so it did not catch the far-field spin in the control; the operator stopped it.

## 6. Where things are

`calibration/yaw_rate_sysid/` (`runs/` baseline, `runs_after/` post-calibration, `reports/before_after_calibration.md`), `calibration/nav_validation/` (`runs/`, `run_b_to_a.sh`, `nav_goal.py`, `analyze_b_to_a.py`, `analyze_partial_run.py`, `kill_sim.sh`, this report in `reports/`). Per-run outputs: `b_to_a_metrics.json`, `b_to_a_overview.png`, `b_to_a_terminal.png`, bag, `gt.csv`, `controller_params.txt`.
