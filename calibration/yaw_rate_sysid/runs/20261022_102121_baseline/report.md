# Yaw-rate system identification: 20261022_102121_baseline

Status: completed  |  ws used by controller: 0.6900 m  |  wheel r (L/R): 0.1000/0.1000
open_loop=False  position_feedback=True  window: hold minus 2.5s start / 0.5s end

## Ratio to COMMAND (source yaw rate / commanded)
| cmd [rad/s] | gt_yaw_slope | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|
| +0.40 | 1.078 | 1.000 | 1.000 | 1.000 | 1.000 | 1.078 | 1.078 | 1.078 | 1.059 |
| -0.40 | 1.078 | 1.000 | 1.000 | 1.000 | 1.000 | 1.078 | 1.078 | 1.078 | 1.059 |
| +0.70 | 1.078 | 1.000 | 1.000 | 1.000 | 1.000 | 1.078 | 1.078 | 1.078 | 1.059 |
| -0.70 | 1.078 | 1.000 | 1.000 | 1.000 | 1.000 | 1.078 | 1.078 | 1.078 | 1.059 |
| +1.00 | 1.078 | 1.000 | 1.000 | 1.000 | 1.000 | 1.078 | 1.078 | 1.078 | 1.059 |
| -1.00 | 1.078 | 1.000 | 1.000 | 1.000 | 1.000 | 1.078 | 1.078 | 1.078 | 1.059 |

## Ratio to GROUND TRUTH (source yaw rate / Gazebo yaw rate)
| cmd [rad/s] | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|
| +0.40 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| -0.40 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| +0.70 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| -0.70 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| +1.00 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| -1.00 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |

## Absolute rates [rad/s]
| cmd | cmd_out | gt_yaw_slope | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|---|
| +0.40 | +0.4000 | +0.4312 | +0.4000 | +0.4000 | +0.4000 | +0.4000 | +0.4312 | +0.4312 | +0.4312 | +0.4235 |
| -0.40 | -0.4000 | -0.4312 | -0.4000 | -0.4000 | -0.4000 | -0.4000 | -0.4312 | -0.4312 | -0.4312 | -0.4235 |
| +0.70 | +0.7000 | +0.7547 | +0.7000 | +0.7000 | +0.7000 | +0.7000 | +0.7547 | +0.7547 | +0.7546 | +0.7411 |
| -0.70 | -0.7000 | -0.7547 | -0.7001 | -0.7000 | -0.7000 | -0.7000 | -0.7547 | -0.7547 | -0.7547 | -0.7412 |
| +1.00 | +1.0000 | +1.0781 | +1.0000 | +1.0000 | +1.0000 | +1.0000 | +1.0781 | +1.0781 | +1.0781 | +1.0588 |
| -1.00 | -1.0000 | -1.0781 | -1.0001 | -1.0000 | -1.0000 | -1.0000 | -1.0781 | -1.0781 | -1.0780 | -1.0587 |

## Speed / sign dependence (ratio to ground truth; gt/cmd is the true tracking ratio)
| |cmd| | gt/cmd (+) | gt/cmd (-) | odom_raw_twist | odom_raw_yaw_slope | wheel_vel | wheel_pos_slope | imu_gyro_z | imu_yaw_slope | fused_twist | fused_yaw_slope |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.40 | 1.078 | 1.078 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| 0.70 | 1.078 | 1.078 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |
| 1.00 | 1.078 | 1.078 | 0.928 | 0.928 | 0.928 | 0.928 | 1.000 | 1.000 | 1.000 | 0.982 |

## Implied effective wheel separation [m] (informational; NOT applied)
| cmd | implied_ws_raw_odom | implied_ws_wheel_pos |
|---|---|---|
| +0.40 | 0.6400 | 0.6400 |
| -0.40 | 0.6400 | 0.6400 |
| +0.70 | 0.6400 | 0.6400 |
| -0.70 | 0.6400 | 0.6400 |
| +1.00 | 0.6400 | 0.6400 |
| -1.00 | 0.6400 | 0.6400 |

Mean over steps: implied_ws_raw_odom=0.6400, implied_ws_wheel_pos=0.6400

## Data checks
sample counts: {'cmd': 5919, 'cmd_out': 13284, 'gt': 10783, 'odom_raw': 7082, 'joints': 13283, 'imu': 13389, 'odom_fused': 7130}
Per-estimate standard errors and sample counts are in `yaw_rate_summary.csv`.