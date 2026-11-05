# Case study 1: yaw-rate mismatch and wheel-separation calibration

*Simulation only. The calibration is specific to this Gazebo model and says nothing about hardware.*

## Problem
Early B→A navigation runs occasionally spun at the goal or stalled. Before tuning the controller, the question was whether the robot
actually turns at the rate it is commanded. If the yaw rate is wrong, every controller and localization result is measured against a
biased plant.

## Investigation
A system-identification harness commanded steady rotations (±0.4, ±0.7, ±1.0 rad/s) and straight lines (±0.3, ±0.6 m/s) and compared
Gazebo ground truth with the commanded rate, the controller's wheel velocities, raw wheel odometry, the IMU and the fused estimate.
The nominal geometry (wheel radius 0.100 m, separation 0.690 m) was checked against the simulator and is realised exactly.

## Measured evidence
| Quantity | Before | After |
|---|---|---|
| True yaw rate / command | 1.07812 (+7.8 %) | 1.00000 |
| Raw wheel-odometry yaw rate / ground truth | 0.92759 (-7.2 %) | 1.00002 |
| Fused (EKF) yaw rate / ground truth | 0.99996 | 1.00000 |
| Fused pose-yaw slope / ground truth | 0.98205 (-1.8 %) | 1.00000 |
| Straight-line velocity / command | 1.00000 | 1.00000 |

Straight-line motion was exact, so the radius is right. Only rotation was off: the simulated contact model behaves as if the wheel
separation were 0.64 m rather than 0.69 m. The identified model reproduces ground-truth yaw rate to 0.0001 % in all six rotation steps.

## Solution
One parameter, `wheel_separation_multiplier = 0.92754` (= 0.64 / 0.69) in `amr_control/config/controllers.yaml`. The URDF geometry was not
edited to hide the discrepancy.

## Validation
The calibration-after run (same six steps) shows the ratios above. In a separate small fixed-start B→A study, 1 of 5 runs ended in a
terminal spin with multiplier 1.0 and 0 of 5 with 0.92754.

## Limitations
* The 5-run comparison is far too small to claim a reliability change; a 95 % interval for 1/5 spins is roughly 0.5 %-72 %.
* The physical cause inside Gazebo's contact model was **not** established; the parameter is an empirical correction.
* A hardware robot needs its own calibration. The simulator number must not be carried over.
* One stall with the calibrated multiplier was not reproduced and remains unexplained.

## Detailed reports
[`calibration/yaw_rate_sysid/reports/before_after_calibration.md`](../../calibration/yaw_rate_sysid/reports/before_after_calibration.md),
[`kinematic_identification_report.md`](../../calibration/yaw_rate_sysid/reports/kinematic_identification_report.md),
[B→A variability study](../../Findings/Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23/Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23.md).
