# Straight-line system identification: 20261022_123234_linear

Status: completed | controller r (L/R) = 0.1/0.1 m | ws = 0.69 | open_loop=False position_feedback=True
Window: hold minus 2.5s start / 0.5s end

## Longitudinal velocity [m/s]
| cmd | cmd_out | GT long | wheel (nominal r) | wheel pos | raw odom twist | raw odom pos | fused twist | fused pos |
|---|---|---|---|---|---|---|---|---|
| +0.30 | +0.3000 | +0.3000 | +0.3000 | +0.3000 | +0.3000 | +0.3000 | +0.3000 | +0.3000 |
| -0.30 | -0.3000 | -0.3000 | -0.3000 | -0.3000 | -0.3000 | -0.3000 | -0.3000 | -0.3000 |
| +0.60 | +0.6000 | +0.6000 | +0.6000 | +0.6000 | +0.6000 | +0.6000 | +0.6000 | +0.6000 |
| -0.60 | -0.6000 | -0.6000 | -0.6000 | -0.6000 | -0.6000 | -0.6000 | -0.6000 | -0.6000 |

## Ratios to command
| cmd | GT | cmd_out | wheel | raw odom twist | fused twist |
|---|---|---|---|---|---|
| +0.30 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| -0.30 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| +0.60 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| -0.60 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

## k_v = GT longitudinal / nominal-radius wheel velocity
| cmd | k_v | raw odom/GT | fused/GT | wheel L [rad/s] | wheel R [rad/s] | (wR-wL)/mean |
|---|---|---|---|---|---|---|
| +0.30 | 1.00000 | 1.0000 | 1.0000 | +3.0000 | +3.0000 | +0.00000 |
| -0.30 | 1.00000 | 1.0000 | 1.0000 | -3.0000 | -3.0000 | -0.00000 |
| +0.60 | 1.00000 | 1.0000 | 1.0000 | +6.0000 | +6.0000 | +0.00000 |
| -0.60 | 1.00000 | 1.0000 | 1.0000 | -6.0000 | -6.0000 | -0.00000 |

k_v mean = 1.00000, std = 0.00000, min/max = 1.00000/1.00000

## Unintended drift in steady-state windows (ground truth + sensors)
| cmd | GT lateral vel [m/s] | lateral dev over window [m] | GT yaw rate [rad/s] | IMU wz | raw odom wz | fused wz | IMU ax |
|---|---|---|---|---|---|---|---|
| +0.30 | -0.00000 | -0.0000 | +0.00000 | +0.00000 | -0.00000 | +0.00000 | +0.00000 |
| -0.30 | +0.00000 | +0.0000 | +0.00000 | -0.00000 | +0.00000 | -0.00000 | -0.00000 |
| +0.60 | -0.00000 | -0.0000 | +0.00000 | +0.00000 | -0.00000 | +0.00000 | +0.00000 |
| -0.60 | +0.00000 | +0.0000 | +0.00000 | +0.00000 | +0.00000 | +0.00000 | -0.00000 |

Whole-run net ground-truth displacement: dx=-0.000 m, dy=-0.000 m, dyaw=+0.0000 rad

sample counts: {'cmd': 3200, 'cmd_out': 7982, 'gt': 6299, 'odom_raw': 4369, 'joints': 7981, 'imu': 7990, 'odom_fused': 4369}