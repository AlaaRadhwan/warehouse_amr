# Yaw-rate system identification: 20261022_123452_rot_wheelwidth_0.10

Status: completed  |  ws used by controller: 0.6900 m  |  wheel r (L/R): 0.1000/0.1000
open_loop=False  position_feedback=True  window: hold minus 2.5s start / 0.5s end

## Ratio to COMMAND (source yaw rate / commanded)
| cmd [rad/s] | gt_yaw_slope | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|
| +0.70 | 1.169 | 1.000 | 1.000 | 1.000 | 1.000 | 1.169 | 1.169 | 1.085 | 1.127 |
| -0.70 | 1.169 | 1.000 | 1.000 | 1.000 | 1.000 | 1.169 | 1.169 | 1.085 | 1.128 |

## Ratio to GROUND TRUTH (source yaw rate / Gazebo yaw rate)
| cmd [rad/s] | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|
| +0.70 | 0.855 | 0.855 | 0.855 | 0.855 | 1.000 | 1.000 | 0.928 | 0.964 |
| -0.70 | 0.855 | 0.855 | 0.855 | 0.855 | 1.000 | 1.000 | 0.928 | 0.964 |

## Absolute rates [rad/s]
| cmd | cmd_out | gt_yaw_slope | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|---|
| +0.70 | +0.7000 | +0.8186 | +0.7000 | +0.7000 | +0.7000 | +0.7000 | +0.8186 | +0.8186 | +0.7595 | +0.7891 |
| -0.70 | -0.7000 | -0.8186 | -0.7000 | -0.7000 | -0.7000 | -0.7000 | -0.8186 | -0.8186 | -0.7598 | -0.7893 |

## Speed / sign dependence (ratio to ground truth; gt/cmd is the true tracking ratio)
| |cmd| | gt/cmd (+) | gt/cmd (-) | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.70 | 1.169 | 1.169 | 0.855 | 0.855 | 0.855 | 0.855 | 1.000 | 1.000 | 0.928 | 0.964 |

## Implied effective wheel separation [m] (informational; NOT applied)
| cmd | implied_ws_raw_odom | implied_ws_wheel_pos |
|---|---|---|
| +0.70 | 0.5900 | 0.5900 |
| -0.70 | 0.5900 | 0.5900 |

Mean over steps: implied_ws_raw_odom=0.5900, implied_ws_wheel_pos=0.5900

## Data checks
sample counts: {'cmd': 1850, 'cmd_out': 5288, 'gt': 4215, 'odom_raw': 2928, 'joints': 5287, 'imu': 5287, 'odom_fused': 2928}
Per-estimate standard errors and sample counts are in `yaw_rate_summary.csv`.