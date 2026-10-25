# AMR effective-kinematics identification

Rotation run: `20261022_102121_baseline`  |  Linear run: `20261022_123234_linear`

## Result

| quantity | value |
|---|---|
| nominal wheel radius | 0.1000 m |
| nominal wheel separation (controller) | 0.6900 m |
| k_v (actual/nominal longitudinal) | 1.00000  (per test: 1.00000, 1.00000, 1.00000, 1.00000) |
| k_w (actual/commanded yaw rate) | 1.07812  (per test: 1.07812, 1.07812, 1.07812, 1.07812, 1.07812, 1.07812) |
| **effective radius** = r k_v | **0.10000 m** |
| **effective separation** = ws k_v / k_w | **0.64000 m** |
| equivalent `wheel_separation_multiplier` (not applied) | 0.92754 |

## Cross-checks (recorded wheel rates + Gazebo ground truth)

- r_eff from v_GT / mean wheel rate, per straight test: 0.10000, 0.10000, 0.10000, 0.10000 m
- ws_eff from r_eff*(wR-wL)/w_GT, per rotation test: 0.64000, 0.64000, 0.64000, 0.64000, 0.64000, 0.64000 m
- body yaw rate predicted from the identified model vs ground truth, per rotation test: +0.0001%, +0.0001%, -0.0000%, -0.0000%, -0.0001%, -0.0001%
- wheel-link centre separation read from Gazebo ground truth: 0.69000 m (nominal geometry is realised exactly)

## Wheel-width diagnostic (scratch models, joint spacing fixed at 0.690 m)

| wheel width [m] | k_w | ws_eff [m] | 0.69 - width [m] | residual [m] | run |
|---|---|---|---|---|---|
| 0.02 | 1.0263 | 0.6723 | 0.6700 | +0.0023 | 20261022_123620_rot_wheelwidth_0.02 |
| 0.05 | 1.0781 | 0.6400 | 0.6400 | +0.0000 | 20261022_102121_baseline |
| 0.10 | 1.1695 | 0.5900 | 0.5900 | +0.0000 | 20261022_123452_rot_wheelwidth_0.10 |