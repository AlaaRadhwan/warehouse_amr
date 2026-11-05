# Demonstration video

`warehouse_amr_demo.mp4` (44 s, 1280×720, no audio) shows the accepted baseline in Gazebo: routed Station B → Station A staging, a rear-obstacle
back-out, and a routed goal with a +90° yaw. **Simulation only; one fresh run per scenario (n = 1); time-lapsed** (3×, 2×, 3×, labelled on screen).

## What was recorded
Launch: `planner:=navfn mppi_path_interpretation:=2d_prefer_forward structured_routing:=true`. Frames come from a temporary Gazebo camera model added to
the running simulation (screen capture was unavailable); the world, map, parameters and routes were not modified. The rear obstacle is a temporary
0.6 × 0.6 × 1.2 m box added at run time, as in the validation test. Clip 1 switches camera once (at 35 s of simulation time) to avoid shelving that blocked the view.

| Clip | Observation in this recording (Gazebo ground truth) |
|---|---|
| 1. Routed B → A staging | reached the goal in 48.1 s; 0.27 m reverse travel; 0.17 m maximum route deviation; −0.3° final yaw error |
| 2. Rear-obstacle back-out | reached the goal; closest footprint-to-box distance about 0.08 m (analytic computation) |
| 3. Routed goal, +90° yaw | reached the goal in 38.8 s; −1.3° final yaw error |

These are new observations, not repeats of the 2026-10-29 validation in [`../CONSOLIDATED_VALIDATION.md`](../CONSOLIDATED_VALIDATION.md); do not mix the two.
A closest distance of a few centimetres is **not** evidence of reliable or hardware-safe obstacle avoidance. The repository's raster clearance script and the
analytic computation used here are different methods and their numbers are not interchangeable.

## Attribution
The world is adapted from the Clearpath Robotics `clearpath_simulator` warehouse world (BSD-3-Clause) and includes Gazebo Fuel models under their own
Creative Commons licences (some NonCommercial/NoDerivatives); see [`../../THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md). This project is independent and
is not affiliated with or endorsed by Clearpath Robotics.
