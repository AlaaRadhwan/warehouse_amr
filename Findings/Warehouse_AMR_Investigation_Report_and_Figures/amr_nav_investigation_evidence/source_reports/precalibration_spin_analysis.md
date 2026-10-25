# B→A terminal spin: offline analysis of `debug/nav_test_b_to_a_bidirectional/`

Read-only analysis. No configuration, source, or rosbag was modified. Nav2 Jazzy 1.3.12 (installed deb); source files
compared against the `1.3.12` git tag (`goal_angle_critic.hpp`, `path_handler.hpp`, `motion_models.hpp` identical to
the installed headers; `.cpp` files could only be taken from the tag).

Time axis: **wall seconds since bag start** (the axis of the earlier t=113–203 s window). Sim = wall x ~0.62.
Controller output ended at wall 196.3 s (goal cancelled by the operator).

## Data limitations
- The bag has **no `/tf`, no action feedback, no action status** messages. map->odom was reconstructed from
  `/transformed_global_plan` single-pose paths (goal yaw 0 rotated into odom).
- The live simulation was no longer running, so runtime parameters were read from the installed
  `nav2_params.yaml` (identical to source), not from `ros2 param`.
- Local costmap and scan were not recorded; per-critic costs were never recorded.

## Confirmed measurements
1. `/plan` and `/unsmoothed_plan` (map frame): 370/370 messages end with yaw exactly 0.0 deg (the goal yaw);
   end xy is the goal (0.099, -0.175) or a point <=5 cm from it (1-pose plans).
2. Robot stays 0.032 m mean / 0.059 m max from the true goal xy for the whole spin (tolerance 0.25 m).
3. `/cmd_vel_nav` angular.z in the window: mean -0.959, 96% of cycles <= -0.5, never above +0.065.
   Between wall 125 and 190 it is -1.0010 +- 0.0009 (pinned at wz_max). Smoothed/final cmd identical.
   Lock-in begins at wall 118.8 s. 9 crossings of the goal yaw, all at full rate; 135 cycles with |yaw error|<15 deg
   all commanded -1.0.
4. Angular velocity at the same time: `/cmd_vel` -1.0010, wheel odometry -1.0012, **IMU gyro -1.0791,
   fused /odom -1.0791**. The body turns ~7.9% faster than commanded; the EKF (IMU-driven yaw rate) reports
   -1.079 rad/s, i.e. beyond `wz_max = 1.0`. map->odom yaw also drifts -0.69 deg/s (AMCL), consistent with a true
   rate slightly above the IMU's.
5. `/optimal_trajectory`: predicted rotation over the 2.8 s horizon averages -121 deg. The end-of-horizon heading
   error is larger than the current one in 67% of cycles (mean 65 -> 98 deg); the selected trajectory rotates
   through the target yaw rather than stopping at it (37% cross it).
6. Transformed path handed to MPPI: single pose in 39% of spin cycles; otherwise 2-12 poses ending 0.05-0.52 m from
   the robot with an arbitrary end yaw (e.g. +71, -106, +76, +50 deg) that changes every replan. 988 of 1010
   multi-pose messages coincide with a cusp in the latest `/plan`.

## Source facts (1.3.12)
- `GoalAngleCritic` target yaw = `data.path.yaws(last)` of the **transformed (cusp-truncated) path**, not the true
  goal yaw; its gate uses the true goal xy (`data.goal` = last pose of the *full* plan).
- `PathAlign`/`PathAngle` return when within `threshold_to_consider` (0.5 m) of the goal; `PathFollow` within 0.10 m;
  `ConstraintCritic` only scores vx; `CostCritic` near the goal only adds critical/collision terms.
  So in the terminal region only GoalCritic, GoalAngleCritic (and Cost/Constraint) act.
- Noise is i.i.d. Gaussian per step (wz sigma 0.4); each sampled velocity is limited to +-az_max*dt (0.05 rad/s) per
  step **starting from the measured speed**; the sequence is then clipped to +-wz_max; Savitzky-Golay is applied in
  place to the persistent sequence; the command is element 1 (shift on).
- `open_loop` is not a string in `libmppi_controller.so` (1.3.12): the `open_loop: false` line is a no-op.

## Reduced model (my reproduction, NOT measured controller cost data)
wz-only copy of the exact update rule above, GoalAngle cost only.
- Plant rate = commanded: converges (fixed goal yaw, 91-94% checker-satisfied; recorded cusp targets: 20%).
- Plant rate = 1.08 x commanded (as measured): **never converges, pinned at wz_max x 1.08**, with or without S-G.
- Threshold: locks when (measured - wz_max) > az_max*dt (gain 1.02 ok, 1.04 locks at az 1.0; 1.04 ok / 1.08 locks at
  az 1.5). Changing wz_max, wz_std or temperature does not help; az_max >= ~2.0 does but exceeds downstream 1.5 limits.
- Mechanism: measured speed above the limit makes every sampled head element > wz_max, so the clipped sequence head
  is always -wz_max; the plant then again exceeds the limit -> fixed point.

## Unresolved
- Why the plant turns 8% faster than the wheel kinematics (0.69 m separation matches URDF; physics/contact or IMU
  scaling). Wheel odometry vs IMU vs AMCL disagree by 8%.
- Whether CostCritic/local-costmap obstacles (not recorded) contribute. Static map is open (>=0.51 m clearance at
  every yaw), so static obstacles are unlikely.
- The cusp-truncated GoalAngle target worsens settling (model: 94% -> 20% checker-satisfied) but did not cause the
  sustained spin in the model.
