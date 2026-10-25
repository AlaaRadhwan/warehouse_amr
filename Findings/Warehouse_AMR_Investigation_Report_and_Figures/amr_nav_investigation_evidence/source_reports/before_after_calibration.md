# Before / after kinematic calibration

before: rotation `20261022_102121_baseline`, linear `20261022_123234_linear`
after:  rotation `20261022_124634_rotation_after`, linear `20261022_124931_linear_after`

Mean over steps, with worst-case |error| in brackets. Error = ratio - 1.

## Rotation (6 steps: +-0.4, +-0.7, +-1.0 rad/s)

| quantity | before | after |
|---|---|---|
| true yaw rate / command | 1.07812 (+7.812%, worst 7.812%) | 1.00000 (+0.000%, worst 0.000%) |
| raw wheel odom twist / GT | 0.92759 (-7.241%, worst 7.242%) | 1.00002 (+0.002%, worst 0.007%) |
| wheel velocities / GT | 0.92754 (-7.246%, worst 7.246%) | 1.00000 (-0.000%, worst 0.000%) |
| IMU gyro / GT | 1.00000 (-0.000%, worst 0.000%) | 1.00000 (-0.000%, worst 0.000%) |
| IMU orientation yaw slope / GT | 1.00000 (+0.000%, worst 0.000%) | 1.00000 (+0.000%, worst 0.000%) |
| fused twist / GT | 0.99996 (-0.004%, worst 0.016%) | 1.00000 (+0.000%, worst 0.001%) |
| fused pose-yaw slope / GT | 0.98205 (-1.795%, worst 1.806%) | 1.00000 (+0.000%, worst 0.001%) |

## Straight line (+-0.3, +-0.6 m/s)

| quantity | before | after |
|---|---|---|
| true velocity / command | 1.00000 (-0.000%, worst 0.000%) | 1.00000 (-0.000%, worst 0.000%) |
| raw odom / GT | 1.00002 (+0.002%, worst 0.003%) | 1.00001 (+0.001%, worst 0.001%) |
| fused twist / GT | 1.00002 (+0.002%, worst 0.003%) | 1.00001 (+0.001%, worst 0.001%) |
| fused pose speed / GT | 1.00002 (+0.002%, worst 0.003%) | 1.00001 (+0.001%, worst 0.002%) |
| k_v = GT / nominal wheel | 1.00000 (-0.000%, worst 0.000%) | 1.00000 (-0.000%, worst 0.000%) |

## Unintended motion in straight-line windows (max |value|)

| | before | after |
|---|---|---|
| GT lateral velocity [m/s] | 9.01e-11 | 8.08e-11 |
| GT yaw rate [rad/s] | 2.97e-13 | 1.32e-15 |