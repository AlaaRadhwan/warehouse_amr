# MTS_AMR — Warehouse Autonomous Mobile Robot (`warehouse_amr`)

A differential-drive warehouse AMR developed in simulation with **ROS 2 Jazzy**, **Nav2** (Smac State Lattice
planner + MPPI controller), **Gazebo** and `ros2_control`'s differential-drive controller.

> **Status: work in progress, simulation only.** Nothing here has been run on hardware, and the navigation stack is
> **not fully validated** (see [Current status](#current-status)). Simulation calibration values are not hardware
> calibration. Nav2's Collision Monitor is not a replacement for independent physical safety measures.

## Packages (`src/`)

| Package | Role |
|---|---|
| `amr_description` | URDF/xacro, meshes, RViz display launch for the robot (including the docking camera mount). |
| `amr_simulation` | Gazebo worlds, models and `simulation.launch.py` (robot spawn, bridges). |
| `amr_control` | `ros2_control` controller configuration (`config/controllers.yaml`: diff-drive controller, `wheel_separation_multiplier`). |
| `amr_localization` | Localization/EKF configuration. |
| `amr_navigation` | Nav2 parameters (`config/nav2_params.yaml`), State Lattice primitives, saved warehouse map, SLAM params, localization+navigation and mapping launch files, RViz config. |
| `amr_interfaces` | Custom actions: `ExecuteMission` (sequence of stations) and `GoToStation`. |
| `amr_missions` | `robot_executor` (station navigation and Nav2 `DockRobot` calls) and `mission_manager` (multi-station missions); stations are defined in `config/stations.yaml` (`home`, `station_a`, `station_b`). |
| `amr_perception` | AprilTag detection config/launch for precision docking at station A. |
| `amr_bringup` | `simulation_navigation.launch.py`: simulation + saved-map localization + Nav2 + robot executor in one launch. |

Several packages still carry placeholder `TODO` descriptions/license declarations in their `package.xml`
(`amr_perception` declares `TODO: License declaration`). Most others declare Apache-2.0.

## Build and run (simulation)

Requirements: Ubuntu with ROS 2 Jazzy, Nav2 (`nav2_smac_planner`, `nav2_mppi_controller`, docking server), Gazebo
(gz-sim) with `ros_gz`, `ros2_control`/`diff_drive_controller`. Install missing dependencies with `rosdep`.

```bash
source /opt/ros/jazzy/setup.bash
cd <repo root>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Gazebo + localization on the saved map + Nav2 + robot executor
ros2 launch amr_bringup simulation_navigation.launch.py
```

The robot spawns at `home`. Goals can be sent to Nav2 (`NavigateToPose`) or through the mission actions of
`amr_missions`. Before launching a new run make sure no orphaned `gz sim` server is still running (they share the
Gazebo transport and corrupt measurements); `calibration/nav_validation/kill_sim.sh` cleans up.
Gazebo ran at a real-time factor of roughly 0.6–0.7 on the development machine; timing-sensitive results depend on it.

## Current status

**Implemented (in source):**
* Gazebo warehouse world, robot model, diff-drive control, saved-map localization, Nav2 bringup.
* Bidirectional navigation baseline: Smac State Lattice with reverse expansion, MPPI (DiffDrive) with reverse
  allowed (`vx_min: -0.35`), path-inversion handling, `StoppedGoalChecker` (0.25 m / 0.20 rad).
* Simulator-specific wheel-separation calibration: `wheel_separation_multiplier: 0.92754` (see `calibration/`).
* Station-based mission executor and a Nav2 `DockRobot` integration; AprilTag detection package.

**Observed but not resolved:** on the Station B → Station A route the robot occasionally rotates in place near the
goal ("spin") or stops short ("stall"). In the instrumented sample, 1 of 5 runs with the uncalibrated multiplier (1.0)
spun and 0 of 5 with 0.92754 failed. This is a small sample and does not establish reliability. A stall seen earlier
in one uninstrumented run with the calibrated multiplier was not reproduced and remains unexplained. See `investigations/btoa_variability_2026-10-23/report/`.

**Not established / not done:** deployment-level reliability of navigation, docking end-to-end validation in this
checkpoint, hardware calibration, recovery policy review, and the planned **Rotation Shim** controller evaluation
(decided in `Findings/MTS_AMR_Navigation_Investigation_Lessons_and_Rotation_Shim_Decision_2026-10-25.md`; **not started**).
`visualize: true` in the MPPI parameters is a diagnostic setting that has not been reverted.

## Where to find the investigations

| Location | Contents |
|---|---|
| `Findings/` | Written reports and the 2026-10-25 lessons / Rotation Shim decision record (start here). |
| `calibration/yaw_rate_sysid/` | Yaw-rate and kinematic identification of the simulator, the 0.92754 multiplier, before/after comparison, scripts and reports. |
| `calibration/nav_validation/` | B → A validation harness (`run_b_to_a.sh`, watchdog, analysis scripts), reports and run records (incl. a run labelled INVALID). |
| `debug/` | Pre-calibration spin analysis (`REPORT.md`), scripts, figures, runtime audit. |
| `investigations/btoa_variability_2026-10-23/` | Instrumented B → A variability study: README, scripts, config, technical report, addendum (MPPI 1.3.12 source check), per-run records and figures. |
| `docs/DATA_MANIFEST.md` | Large/derived data kept **out of Git** (rosbags, pickles, metrics): paths, sizes, SHA-256, regeneration steps. |

Raw rosbags (`*.mcap`, ~0.7 GB) are intentionally not in Git and need an independent backup — see the manifest.

## Working agreement

`CLAUDE.md` defines how the owner and the AI assistant divide work (owner: requirements, architecture, behavior,
core new functionality; assistant: integration, diagnostics, tests, review) and the repository-protection rules.

## Licenses and third-party material

Project packages are Apache-2.0 where declared. `investigations/btoa_variability_2026-10-23/analysis/addendum/mppi_1.3.12_source_used/`
holds read-only reference copies of 19 `nav2_mppi_controller` 1.3.12 files (Apache-2.0, Samsung Research America /
the Nav2 contributors) with a license copy and notice; they are not built.
