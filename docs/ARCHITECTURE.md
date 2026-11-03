# Architecture

Simulation-only differential-drive warehouse AMR. Facts below come from the launch files and sources on this branch; items not
exercised by the one-command launch are marked.

## Packages
| Package | Role |
|---|---|
| `amr_description` | URDF/xacro robot model (primitive shapes; LiDAR, IMU, docking camera). |
| `amr_control` | `ros2_control` diff-drive configuration (`wheel_separation_multiplier: 0.92754`, simulator calibration). |
| `amr_localization` | `robot_localization` EKF: wheel odometry (`/diff_drive_controller/odom`) + IMU (`/imu/data_raw`), 50 Hz. |
| `amr_simulation` | Gazebo worlds, AprilTag model, ROS↔Gazebo bridge (`/clock`, `/scan`, `/imu/data_raw`, camera topics), simulation launch. |
| `amr_navigation` | Nav2 parameters, profile overlays, saved map, State Lattice primitives, route graph, route behavior tree. |
| `amr_interfaces` | Actions `ExecuteMission` (list of stations) and `GoToStation`. |
| `amr_missions` | `robot_executor` (`GoToStation` server; calls Nav2 `NavigateToPose` and `DockRobot`) and `mission_manager` (`ExecuteMission` server; calls `GoToStation`). Stations in `config/stations.yaml`. |
| `amr_perception` | `apriltag_ros` configuration and `dock_pose_publisher` for Station A docking. |
| `amr_bringup` | `simulation_navigation.launch.py`: composes the packages above and applies the planner/controller/routing profiles. |

## Launch composition
`amr_bringup/simulation_navigation.launch.py` includes, in order:
1. `amr_simulation/simulation.launch.py`: robot_state_publisher, `gz sim` with the warehouse world, `ros_gz_bridge`, robot spawn, `joint_state_broadcaster` and `diff_drive_controller` spawners, EKF.
2. `amr_navigation/localization_navigation.launch.py`: Nav2 `bringup_launch.py` (map server, AMCL, planner, controller, BT navigator, route server, velocity smoother, collision monitor, docking server) and RViz.
3. `amr_missions/robot_executor.launch.py`: `robot_executor` and `mission_manager`.

**Not in that launch:** `amr_perception/launch/apriltag.launch.py` (AprilTag detection and dock pose). Station A precision docking therefore
needs that launch started separately; the end-to-end docking run has not been re-validated on the accepted baseline.

## Navigation data flow (accepted baseline profile)
Goal (`NavigateToPose`, directly or via the mission layer) → BT navigator → **route behavior tree**: Route Server computes the lane path;
guarded first/last-mile planning (NavFn, ≤ 1.5 m) → **MPPI** controller (2D-path interpretation, reverse allowed, `PreferForwardCritic`) →
velocity smoother → collision monitor → `cmd_vel` → `diff_drive_controller`. Localization: AMCL on the saved map, EKF for odometry.
Profiles are YAML overlays merged over `nav2_params.yaml` at launch; the launch defaults still start the earlier Smac lattice + MPPI stack.

## Frames and ground truth
`map` → `odom` (AMCL) → `base_footprint` (EKF). Validation metrics in the reports use Gazebo ground truth, kept distinct from wheel odometry
and from the fused/localized estimates.

See [`NAVIGATION_BASELINE.md`](NAVIGATION_BASELINE.md) for configuration details and [`LIMITATIONS_AND_FUTURE_WORK.md`](LIMITATIONS_AND_FUTURE_WORK.md).
