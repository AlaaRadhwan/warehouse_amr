# Warehouse AMR

**Independent ROS 2 autonomous mobile robot project: calibrated differential-drive kinematics, lane-based routing and MPPI control for a warehouse robot in Gazebo.**

A differential-drive warehouse robot built on ROS 2 Jazzy and Nav2. The project calibrates the simulated wheel kinematics, routes the robot along
warehouse lanes with the Nav2 Route Server, controls it with MPPI that prefers forward motion, and exposes station-based missions. The reports behind
each design decision are kept in the repository.

> **Simulation only — not tested on hardware.**

## Demonstration
Routed navigation between stations, backing out of a tight spot with an obstacle behind the robot, and arriving at a goal with a requested orientation
(time-lapsed Gazebo footage of the accepted configuration; details in [`docs/media/README.md`](docs/media/README.md)).

https://github.com/user-attachments/assets/8e7df18d-d176-4938-9d69-d698aadf52a2

## System architecture
![Warehouse AMR system architecture](docs/figures/architecture.svg)

A goal arrives from the mission layer or directly as a `NavigateToPose` request. The behavior tree coordinates the **Route Server** (lane-based route)
and **NavFn** (first and last mile, which preserves the requested goal orientation) and hands the resulting path to **MPPI**. Velocity commands pass through
a smoother and a collision monitor to the `ros2_control` differential-drive controller in Gazebo. An EKF fuses wheel odometry and IMU, and AMCL localizes
against a saved occupancy map. More detail: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Technology stack and navigation approach
**Stack:** ROS 2 Jazzy, Nav2, Gazebo Harmonic, Python, `ros_gz`, `ros2_control`, `robot_localization`, AMCL, and `apriltag_ros`.

**Accepted navigation configuration:**
* **Routing:** Nav2 Route Server on a lane graph, with a custom route behavior tree that always plans the last mile so the goal orientation is honored.
* **Planning:** NavFn for the first and last mile.
* **Control:** MPPI using the 2D path (path yaw is not treated as a heading), reverse allowed (`vx_min -0.35`), and `PreferForwardCritic` (weight 5.0) to keep reversing to short back-outs.
* **Kinematics:** wheel-separation multiplier 0.92754, a simulator calibration.
* **Goal checker:** `StoppedGoalChecker` (0.25 m, 0.20 rad).

Full parameter table and rationale: [`docs/NAVIGATION_BASELINE.md`](docs/NAVIGATION_BASELINE.md).

| Package (`src/`) | Role |
|---|---|
| `amr_description`, `amr_control` | robot model; `ros2_control` differential drive |
| `amr_simulation`, `amr_localization` | Gazebo world and bridge; EKF |
| `amr_navigation` | Nav2 parameters and profiles, map, route graph, route behavior tree |
| `amr_missions`, `amr_interfaces` | `GoToStation` and `ExecuteMission` actions and executors |
| `amr_perception` | AprilTag configuration and dock pose publisher |
| `amr_bringup` | launch file composing the above |

## Installation and usage
Tested on Ubuntu 24.04 with ROS 2 Jazzy. The repository root is the colcon workspace (packages are in `src/`). The simulated world downloads its models from Gazebo Fuel on first run.
```bash
git clone https://github.com/AlaaRadhwan/warehouse_amr.git
cd warehouse_amr                      # all commands below run from here
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash             # in every new terminal

ros2 launch amr_bringup simulation_navigation.launch.py \
  planner:=navfn mppi_path_interpretation:=2d_prefer_forward structured_routing:=true
```
Wait until the Nav2 lifecycle manager reports "Managed nodes are active", then send goals from a second terminal (source the workspace first). Stations are `home`, `station_a`, `station_b`.
```bash
# Go to a station through the mission layer
ros2 action send_goal /mission/go_to_station amr_interfaces/action/GoToStation "{station_name: station_b}"

# Go to an arbitrary pose (map frame, yaw 90 deg) with the route behavior tree
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map},
  pose: {position: {x: 8.716, y: 2.705}, orientation: {z: 0.7071, w: 0.7071}}},
  behavior_tree: '$(ros2 pkg prefix amr_navigation)/share/amr_navigation/behavior_trees/navigate_on_warehouse_route.xml'}"
```
Running the launch file with no arguments starts the earlier State Lattice configuration instead. `station_a` is a docking station; docking is described under [Known limitations](#known-limitations).

## Engineering investigations
1. [**Yaw-rate mismatch and wheel-separation calibration**](docs/case_studies/01_yaw_rate_calibration.md): the simulated robot turned 7.8 % faster than commanded; one calibrated parameter (0.92754) removed the mismatch.
2. [**Planner and MPPI evaluation, and reverse-navigation policy**](docs/case_studies/02_planner_mppi_reverse_policy.md): investigated unwanted reverse motion caused by path-orientation interpretation, then used a forward preference to reduce reverse travel from 18 m to 0.24 m in the tested scenario.
3. [**Route Server goal-orientation failure**](docs/case_studies/03_route_server_goal_orientation.md): the project's route behavior tree skipped last-mile planning when the goal sat on the route end, so a wrong-orientation arrival was reported as success; a one-block fix in that tree restored the requested orientation.

## Selected experimental results
Accepted configuration, simulation, one run per test (2026-10-29), measured with Gazebo ground truth:

| Test | Result |
|---|---|
| Routed Station B → A staging | 48.4 s, 0.14 m maximum route deviation, 0.24 m reverse travel |
| Free-space Station B → Home | 48.2 s, 0.71 m reverse travel |
| Back out of a pocket with an obstacle behind | reached the goal; 0.05 m closest footprint margin, which is tight |
| Routed goal with +90° requested yaw | reached the goal, −3.5° yaw error |

Results are from individual simulation runs (n=1), not reliability measurements. Details: [`docs/CONSOLIDATED_VALIDATION.md`](docs/CONSOLIDATED_VALIDATION.md).

## Known limitations
* The route graph is a provisional six-node fixture.
* Docking and AprilTag detection are implemented in source but have not been re-validated on the accepted configuration (the AprilTag launch is separate).
* A Rotation Shim controller evaluation on the separate branch `feature/rotation-shim-integration` is experimental and unfinished; it is not part of the accepted configuration.

Full list and planned work: [`docs/LIMITATIONS_AND_FUTURE_WORK.md`](docs/LIMITATIONS_AND_FUTURE_WORK.md).

## Documentation and attribution
[`docs/README.md`](docs/README.md) indexes the architecture, navigation baseline, validation reports, case studies and the historical investigation reports in [`Findings/`](Findings/README.md).
Raw rosbags are not stored in Git ([`docs/DATA_MANIFEST.md`](docs/DATA_MANIFEST.md)).

Developed with AI assistance (Claude Code); commits carry co-author trailers, and [`CLAUDE.md`](CLAUDE.md) records how design ownership and implementation were divided.

**License:** original code, configuration and documentation are under the [Apache License 2.0](LICENSE), Copyright 2026 Alaa Radhwan Mustafa Shamsan ([`NOTICE`](NOTICE)).

**Third-party material:** the warehouse world is adapted from the Clearpath Robotics [`clearpath_simulator`](https://github.com/clearpathrobotics/clearpath_simulator) warehouse world
(BSD-3-Clause; [notice](src/amr_simulation/worlds/NOTICE-clearpath.md)). The building, shelves and props are Gazebo Fuel models by other authors under their own Creative Commons licences
(some NonCommercial/NoDerivatives), downloaded at run time and shown in the images and video. See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
This project is independent and is not affiliated with or endorsed by Clearpath Robotics.
