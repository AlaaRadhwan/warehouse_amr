# Yaw-rate system identification: 20261022_123620_rot_wheelwidth_0.02

Status: completed  |  ws used by controller: 0.6900 m  |  wheel r (L/R): 0.1000/0.1000
open_loop=False  position_feedback=True  window: hold minus 2.5s start / 0.5s end

## Ratio to COMMAND (source yaw rate / commanded)
| cmd [rad/s] | gt_yaw_slope | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|
| +0.70 | 1.026 | 1.000 | 1.000 | 1.000 | 1.000 | 1.026 | 1.026 | 1.026 | 1.020 |
| -0.70 | 1.026 | 1.000 | 1.000 | 1.000 | 1.000 | 1.026 | 1.026 | 1.026 | 1.020 |

## Ratio to GROUND TRUTH (source yaw rate / Gazebo yaw rate)
| cmd [rad/s] | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|
| +0.70 | 0.975 | 0.975 | 0.975 | 0.975 | 1.000 | 1.000 | 1.000 | 0.994 |
| -0.70 | 0.974 | 0.974 | 0.974 | 0.974 | 0.999 | 1.000 | 0.999 | 0.994 |

## Absolute rates [rad/s]
| cmd | cmd_out | gt_yaw_slope | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|---|
| +0.70 | +0.7000 | +0.7183 | +0.7000 | +0.7000 | +0.7000 | +0.7000 | +0.7185 | +0.7183 | +0.7185 | +0.7137 |
| -0.70 | -0.7000 | -0.7185 | -0.7000 | -0.7000 | -0.7000 | -0.7000 | -0.7181 | -0.7185 | -0.7180 | -0.7139 |

## Speed / sign dependence (ratio to ground truth; gt/cmd is the true tracking ratio)
| |cmd| | gt/cmd (+) | gt/cmd (-) | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.70 | 1.026 | 1.026 | 0.974 | 0.974 | 0.974 | 0.974 | 1.000 | 1.000 | 1.000 | 0.994 |

## Implied effective wheel separation [m] (informational; NOT applied)
| cmd | implied_ws_raw_odom | implied_ws_wheel_pos |
|---|---|---|
| +0.70 | 0.6724 | 0.6724 |
| -0.70 | 0.6722 | 0.6722 |

Mean over steps: implied_ws_raw_odom=0.6723, implied_ws_wheel_pos=0.6723

## Data checks
sample counts: {'cmd': 1850, 'cmd_out': 5202, 'gt': 4370, 'odom_raw': 2967, 'joints': 5200, 'imu': 5284, 'odom_fused': 3009}
Per-estimate standard errors and sample counts are in `yaw_rate_summary.csv`.