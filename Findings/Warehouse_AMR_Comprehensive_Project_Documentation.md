# Warehouse AMR Project — Comprehensive Technical Documentation

**Project:** Warehouse AMR / `warehouse_amr` (written under the earlier working name "MTS_AMR")  
**Repository:** `AlaaRadhwan/warehouse_amr`  
**Primary OS:** Ubuntu 24.04  
**ROS distribution:** ROS 2 Jazzy  
**Simulation:** Gazebo Harmonic (`gz sim`)  
**Document snapshot:** 2026-10-14  
**Known-good Git checkpoint:** `14608840c994858edfd9c05c89321c9d8ebd6069` — `Add AprilTag precision docking for station A`

---

## 1. Project Purpose

This project develops a warehouse autonomous mobile robot (AMR) using a simulation-first architecture that can later be transferred to a real differential-drive platform.

The current vehicle concept is:

- approximately 1.0 m long and 0.60 m wide,
- differential drive with two powered wheels,
- four passive support/caster wheels,
- intended real payload capacity of approximately 50 kg,
- 2D LiDAR for mapping, localization and obstacle sensing,
- IMU fused with wheel odometry,
- forward/downward camera for precision station docking,
- ROS 2 Nav2 for warehouse navigation,
- AprilTags for local station-relative pose refinement,
- a mission layer above Nav2,
- later separation between ROS 2 autonomy and an industrial PLC / drive-control layer.

The main architectural principle is to separate four responsibilities:

1. **Global warehouse navigation** — LiDAR, AMCL/SLAM, Nav2 planner/controller.
2. **Local station alignment** — AprilTag perception and Nav2 Docking Server.
3. **Mission logic** — named stations and station sequences rather than raw poses.
4. **Low-level machine control** — currently `ros2_control` in Gazebo; intended to become a real PLC/drive interface later.

The current system has successfully demonstrated AprilTag-refined precision docking at `station_a` using Nav2's Docking Server.

---

## 2. Current Verified Capabilities

| Capability | Status |
|---|---|
| Differential-drive Gazebo model | Working |
| `ros2_control` wheel control | Working |
| Wheel odometry | Working |
| IMU | Working |
| 2D LiDAR | Working |
| EKF odometry fusion | Working |
| SLAM Toolbox mapping | Working |
| Saved-map localization with AMCL | Working |
| Nav2 global planning | Working |
| Nav2 local control with Regulated Pure Pursuit | Working |
| Local and global obstacle costmaps | Working |
| Dynamic obstacle marking/clearing | Working |
| Velocity smoother | Working |
| Nav2 collision monitor | Working in simulation; not safety-rated |
| Named station configuration | Working |
| `GoToStation` custom action | Working |
| Multi-station mission sequencing | Working |
| Simulated docking camera | Working |
| AprilTag 36h11 detection | Working |
| AprilTag-derived dock pose | Working |
| Nav2 Docking Server precision approach | Working |
| `DockRobot` to `station_a` | Verified successful, zero retries |
| Station B precision docking | Not yet implemented |
| Home precision docking | Not yet implemented |
| Automatic mission-layer use of `DockRobot` | Not yet implemented |
| Multi-robot fleet layer | Not yet implemented |
| Real PLC / BLDC drive interface | Not yet implemented |

A successful docking test returned:

```text
Result:
    success: true
error_code: 0
num_retries: 0
error_msg: ''

Goal finished with status: SUCCEEDED
```

---

## 3. System Architecture

### 3.1 Current navigation and mission architecture

```text
                         +---------------------+
                         |   MissionManager    |
                         | ExecuteMission      |
                         +----------+----------+
                                    |
                                    | GoToStation
                                    v
                         +---------------------+
                         |   RobotExecutor     |
                         +----------+----------+
                                    |
                  currently         | NavigateToPose
                                    v
+-------------+          +-----------------------------+
| stations.yaml|         |            Nav2             |
+-------------+          | planner / controller / AMCL |
                         +-----------+-----------------+
                                     |
                                     | velocity command chain
                                     v
                         +-----------------------------+
                         | smoother / collision layer  |
                         +-------------+---------------+
                                       |
                                       v
                             +------------------+
                             | diff_drive_ctrl  |
                             +--------+---------+
                                      |
                                      v
                                  Gazebo AMR
```

### 3.2 Current precision docking architecture

```text
DockRobot(station_a)
        |
        v
nominal dock database pose
        |
        v
Navigate to staging pose
        |
        v
AprilTag becomes visible
        |
        v
apriltag_ros
        |
        v
tag_station_a TF
        |
        v
dock_pose_publisher
        |
        v
/detected_dock_pose
        |
        v
Nav2 Docking Server
        |
        v
precision velocity control
        |
        v
final station pose
```

The next integration step is to make `RobotExecutor` call `DockRobot` automatically for dock-enabled stations, so applications only request a station name.

### 3.3 Intended real-hardware architecture

```text
                  ROS 2 COMPUTER
          Raspberry Pi 5 / industrial IPC
        +-------------------------------+
        | Nav2                          |
        | localization / SLAM           |
        | perception                    |
        | mission logic                 |
        | fleet integration             |
        +---------------+---------------+
                        |
              defined drive interface
                        |
                        v
                 SIEMENS PLC
               S7-1500 family
        +-------------------------------+
        | deterministic machine logic   |
        | watchdogs                     |
        | drive command interface       |
        | machine I/O                   |
        +---------------+---------------+
                        |
                        v
               BLDC motor drives
                        |
                        v
                 drive motors
```

Safety functions such as E-stop, safety scanner inputs and STO must remain independent of ordinary ROS navigation software. Nav2 Collision Monitor is useful for operational collision prevention but is **not a certified safety function**.

---

## 4. Repository Structure

```text
warehouse_amr/
├── src/
│   ├── amr_bringup/
│   │   └── launch/
│   │       └── simulation_navigation.launch.py
│   │
│   ├── amr_control/
│   │   └── config/
│   │       └── controllers.yaml
│   │
│   ├── amr_description/
│   │   ├── launch/
│   │   └── urdf/
│   │       ├── amr.urdf.xacro
│   │       └── amr_properties.xacro
│   │
│   ├── amr_interfaces/
│   │   └── action/
│   │       ├── GoToStation.action
│   │       └── ExecuteMission.action
│   │
│   ├── amr_localization/
│   │   └── config/
│   │       └── ekf.yaml
│   │
│   ├── amr_missions/
│   │   ├── amr_missions/
│   │   │   ├── robot_executor.py
│   │   │   └── mission_manager.py
│   │   ├── config/
│   │   │   └── stations.yaml
│   │   └── launch/
│   │       └── robot_executor.launch.py
│   │
│   ├── amr_navigation/
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   └── slam.yaml
│   │   ├── launch/
│   │   │   ├── localization_navigation.launch.py
│   │   │   └── mapping_navigation.launch.py
│   │   └── maps/
│   │       ├── warehouse.pgm
│   │       └── warehouse.yaml
│   │
│   ├── amr_perception/
│   │   ├── amr_perception/
│   │   │   └── dock_pose_publisher.py
│   │   ├── config/
│   │   │   └── apriltags.yaml
│   │   └── launch/
│   │       └── apriltag.launch.py
│   │
│   └── amr_simulation/
│       ├── config/
│       │   └── bridge.yaml
│       ├── launch/
│       │   └── simulation.launch.py
│       ├── models/
│       │   └── apriltag_station_a/
│       └── worlds/
│           ├── empty_world.sdf
│           └── warehouse_world.sdf
│
├── build/       # generated; ignored by Git
├── install/     # generated; ignored by Git
└── log/         # generated; ignored by Git
```

### Package responsibilities

**`amr_description`** owns the URDF/Xacro robot model: chassis, wheels, passive supports, sensor frames, camera frames, simulation sensors and Gazebo `ros2_control` hardware definition.

**`amr_control`** contains `ros2_control` controller configuration, differential-drive geometry and motion limits.

**`amr_localization`** contains the `robot_localization` EKF configuration.

**`amr_navigation`** owns Nav2, AMCL, SLAM Toolbox, costmaps, planner/controller configuration, docking configuration and saved maps.

**`amr_simulation`** owns Gazebo launch, the warehouse world, ROS/Gazebo bridges and custom simulation models such as the Station A AprilTag.

**`amr_interfaces`** defines project-level ROS actions.

**`amr_missions`** owns named stations, `RobotExecutor` and `MissionManager`.

**`amr_perception`** owns AprilTag perception and conversion from a raw tag pose into the robot pose expected by the docking controller.

**`amr_bringup`** composes the main simulation/navigation/mission launch.

---

## 5. Development Environment

Known working environment:

```text
Ubuntu:        24.04
ROS 2:        Jazzy
Gazebo:       Harmonic / gz sim 8.x
Workspace:    ~/Projects/warehouse_amr
```

Shell setup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/warehouse_amr/install/setup.bash
```

Build the full workspace:

```bash
cd ~/Projects/warehouse_amr
colcon build --symlink-install
source install/setup.bash
```

Build a changed package only:

```bash
colcon build --symlink-install --packages-select amr_navigation
```

or:

```bash
colcon build --symlink-install --packages-select amr_perception
```

Important ROS components include:

- Nav2 / `nav2_bringup`,
- SLAM Toolbox,
- `robot_localization`,
- `ros2_control`,
- `ros2_controllers`,
- `gz_ros2_control`,
- `ros_gz_sim`,
- `ros_gz_bridge`,
- `xacro`,
- `robot_state_publisher`,
- RViz 2,
- `apriltag_ros`,
- `apriltag_msgs`.

AprilTag package used in the project:

```bash
sudo apt install ros-jazzy-apriltag-ros
```

On a new machine, use `rosdep` where possible:

```bash
cd ~/Projects/warehouse_amr
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Current limitation:** several package manifests remain minimal and do not yet declare every runtime dependency. Manifest cleanup should be completed before expecting `rosdep` alone to reproduce the entire environment.

---

## 6. Robot Model

### 6.1 Coordinate convention

```text
+x = forward
+y = left
+z = up
```

Main TF hierarchy:

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── left_wheel_link
                ├── right_wheel_link
                ├── support wheel links
                ├── imu_link
                ├── lidar_link
                └── camera_mount_link
                     └── camera_link
                          └── camera_optical_frame
```

`camera_optical_frame` follows the ROS optical convention:

```text
+x = right
+y = down
+z = forward
```

### 6.2 Main geometry

| Parameter | Value |
|---|---:|
| Collision envelope length | 1.00 m |
| Collision envelope width | 0.60 m |
| Collision envelope height | 0.20 m |
| Base mass parameter | 50 kg |
| Ground clearance | 0.06 m |
| Drive wheel radius | 0.10 m |
| Drive wheel width | 0.05 m |
| Wheel centerline separation | 0.69 m |
| Drive wheel mass | 2.0 kg each |
| Passive support radius | 0.04 m |
| Passive support mass | 0.5 kg each |
| Support longitudinal location | ±0.40 m |
| Support lateral location | ±0.24 m |

The intended real platform may carry up to roughly 50 kg payload. Payload dynamics are not yet represented as a variable load in simulation.

The visual AMR includes a lower enclosure, payload deck, front sensor nose, side fairings, bumpers, LiDAR pedestal and protected camera housing. The collision model intentionally remains simpler and conservative.

---

## 7. Sensors

### 7.1 LiDAR

Current simulated LiDAR:

```text
Type:           GPU 2D lidar
Samples:        720
FOV:            360°
Update rate:    10 Hz simulation time
Minimum range:  0.05 m
Maximum range:  20.0 m
Topic:          /scan
Frame:          lidar_link
```

LiDAR position relative to `base_link`:

```text
x = +0.35 m
y =  0.00 m
z = +0.14 m
```

Since `base_link` is approximately 0.16 m above the floor, LiDAR height is around 0.30 m.

LiDAR is used by AMCL, SLAM Toolbox, obstacle costmaps and Collision Monitor.

### 7.2 IMU

```text
Topic:       /imu/data_raw
Frame:       imu_link
Update rate: 100 Hz
```

Current EKF use is primarily yaw-rate fusion together with wheel odometry.

### 7.3 Docking camera

The camera geometry is based on a future Raspberry Pi Global Shutter Camera / Sony IMX296-style installation with a provisional C/CS lens.

Current simulation settings:

```text
Resolution:      640 x 480
Format:          R8G8B8
Update rate:     10 Hz simulation time
HFOV:            0.7924 rad ≈ 45.4°
Near clip:       0.05 m
Far clip:        8.0 m
Image topic:     /camera/image_raw
CameraInfo:      /camera/camera_info
Optical frame:   camera_optical_frame
always_on:       false
visualize:       false
```

Camera mount relative to `base_link`:

```text
x = 0.430 m
y = 0
z = 0.080 m
pitch = 25° downward
```

`camera_link` is another approximately 0.035 m forward from the mount origin. The optical center is therefore around 0.23–0.24 m above the floor.

The camera is angled forward/downward instead of straight down so a floor marker remains visible through the final approach.

---

## 8. Gazebo Simulation

### 8.1 Physics

Current world physics:

```xml
<physics name="spot_physics" type="dart">
  <max_step_size>0.0005</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

This is a 0.5 ms physics step, nominally 2 kHz. It is likely more demanding than necessary for a differential-drive AMR but is currently retained because the navigation/docking baseline is working. Later performance work can test 1 ms or 2 ms.

### 8.2 Warehouse world

`warehouse_world.sdf` contains:

- Gazebo Fuel warehouse model,
- shelves,
- barriers,
- chairs/tables,
- simulated people,
- Station A AprilTag target.

The main bringup spawns the robot at the configured `home` station.

### 8.3 ROS ↔ Gazebo bridge

`bridge.yaml` bridges:

```text
/clock
/scan
/imu/data_raw
/camera/image_raw
/camera/camera_info
```

Camera bridges use:

```yaml
lazy: true
```

This reduces rendering/transport load when no camera consumer is active.

### 8.4 Camera performance lesson

An earlier camera configuration near:

```text
1456 x 1088 @ 30 Hz
```

caused severe rendering load and affected timely Nav2/costmap startup.

The current stable baseline is:

```text
640 x 480 @ 10 Hz
visualize = false
lazy bridge = true
```

If Gazebo runs at 0.7 real-time factor, a 10 Hz simulated sensor may appear near 7 Hz in wall-clock `ros2 topic hz`. That does not by itself indicate packet loss.

---

## 9. Drive Control

`amr_control/config/controllers.yaml` configures:

```text
controller_manager update rate: 100 Hz
diff_drive_controller publish rate: 50 Hz
```

Drive geometry:

```text
wheel radius:      0.10 m
wheel separation:  0.69 m
```

Motion limits:

```text
linear velocity:       ±1.0 m/s
linear acceleration:    0.8 m/s²
angular velocity:      ±1.2 rad/s
angular acceleration:  ±1.5 rad/s²
```

The differential-drive controller publishes wheel odometry but does not publish `odom -> base_footprint`:

```yaml
enable_odom_tf: false
```

The EKF owns that transform.

---

## 10. Localization and TF Ownership

### 10.1 EKF

`robot_localization` configuration:

```text
frequency:       50 Hz
two_d_mode:      true
world_frame:     odom
odom_frame:      odom
base_link_frame: base_footprint
publish_tf:      true
```

Inputs:

```text
/diff_drive_controller/odom
/imu/data_raw
```

Outputs:

```text
/odom
odom -> base_footprint
```

### 10.2 Global localization

Saved-map operation:

```text
AMCL owns: map -> odom
```

Mapping operation:

```text
SLAM Toolbox owns: map -> odom
```

Normal TF chain:

```text
map
  |
  | AMCL or SLAM Toolbox
  v
odom
  |
  | EKF
  v
base_footprint
  |
  | robot_state_publisher
  v
base_link and sensor frames
```

Do not add another publisher for either transform unless the architecture is intentionally changed.

---

## 11. Mapping

Configuration:

```text
src/amr_navigation/config/slam.yaml
```

Important values:

```text
map frame:        map
odom frame:       odom
base frame:       base_footprint
scan topic:       /scan
resolution:       0.05 m
mode:             mapping
loop closing:     enabled
scan matching:    enabled
use_sim_time:     true
```

Mapping launch:

```bash
ros2 launch amr_navigation mapping_navigation.launch.py
```

This launches SLAM Toolbox, Nav2 without AMCL/static-map localization and RViz.

Saved map:

```text
src/amr_navigation/maps/warehouse.pgm
src/amr_navigation/maps/warehouse.yaml
```

---

## 12. Saved-Map Navigation and Main Bringup

Saved-map navigation:

```bash
ros2 launch amr_navigation localization_navigation.launch.py
```

It uses:

- `warehouse.yaml`,
- AMCL,
- Nav2,
- RViz.

Main integrated simulation launch:

```bash
ros2 launch amr_bringup simulation_navigation.launch.py
```

That launch:

1. loads `stations.yaml`,
2. reads the `home` station,
3. rewrites AMCL initial pose at runtime,
4. spawns Gazebo robot at the same home pose,
5. launches saved-map Nav2,
6. launches `RobotExecutor`,
7. launches `MissionManager`.

This keeps simulated robot spawn and AMCL initial pose consistent.

---

## 13. Stations

Current station registry:

```yaml
frame_id: map

stations:

  home:
    x: -7.589
    y: 1.2
    yaw: 0.0

  station_a:
    x: 0.799
    y: -0.175
    yaw: 0.0

  station_b:
    x: 8.716
    y: 2.705
    yaw: 0.0
```

Source:

```text
src/amr_missions/config/stations.yaml
```

All current station headings are `yaw = 0`, so final robot forward is map `+X`.

At present, `stations.yaml` is the mission-level source of station positions and the source used by bringup for the Home spawn pose.

**Temporary duplication:** `station_a` is also hard-coded in the Nav2 docking database for the first docking proof. This is intentional only for the current integration stage and should be removed next.

---

## 14. Nav2 Configuration

Primary file:

```text
src/amr_navigation/config/nav2_params.yaml
```

### 14.1 Planner

Current global planner:

```text
nav2_smac_planner::SmacPlanner2D
```

Relevant values:

```text
allow_unknown:              true
planning tolerance:         0.2 m
max planning time:          2.0 s
cost travel multiplier:     2.0
final approach orientation: disabled
```

### 14.2 Local controller

Current local controller:

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

Important values:

```text
desired linear velocity:             0.8 m/s
lookahead distance:                  0.8 m
minimum lookahead:                   0.5 m
maximum lookahead:                   1.2 m
lookahead time:                      1.5 s
velocity-scaled lookahead:           enabled

rotate-to-heading:                   enabled
rotate-to-heading angular velocity:  0.45 rad/s
rotate threshold:                    0.6 rad
max angular acceleration:            1.0 rad/s²

allow reversing:                     false

minimum approach velocity:           0.05 m/s
approach scaling distance:           0.8 m

regulated curvature scaling:         enabled
minimum scaling radius:              0.9 m
minimum regulated speed:             0.15 m/s

cost-based velocity scaling:         disabled
controller collision detection:      enabled
TTC limit:                            1.2 s
```

Goal checker:

```text
XY tolerance:   0.10 m
yaw tolerance:  0.05 rad
```

RPP replaced an earlier MPPI setup because the previous controller configuration repeatedly produced undesirable terminal-yaw behavior, including large unnecessary rotations near the goal.

Ordinary Nav2 is not expected to provide final millimetre-level workstation alignment; precision alignment is delegated to docking.

---

## 15. Costmaps

### 15.1 Local costmap

```text
frame:               odom
rolling window:      true
size:                7 m x 7 m
resolution:          0.05 m
update frequency:    10 Hz
publish frequency:   5 Hz
```

Plugins:

```text
ObstacleLayer
InflationLayer
```

Robot footprint:

```text
[(+0.50,+0.37),
 (+0.50,-0.37),
 (-0.50,-0.37),
 (-0.50,+0.37)]
```

Obstacle settings:

```text
obstacle range:        0.05 to 8.0 m
raytrace range:        0.05 to 10.0 m
obstacle height:       0.0 to 2.0 m
inflation radius:      0.70 m
cost scaling factor:   3.0
```

### 15.2 Global costmap

```text
frame:               map
resolution:          0.05 m
track unknown:       true
update frequency:    2 Hz
publish frequency:   1 Hz
```

Plugins:

```text
StaticLayer
ObstacleLayer
InflationLayer
```

---

## 16. Velocity Command Pipeline

The project uses stamped velocity commands.

Relevant Nav2 components are configured with:

```yaml
enable_stamped_cmd_vel: true
```

This includes the Docking Server.

This setting is critical. During initial docking integration, Docking Server was left in unstamped mode. It successfully:

- navigated to staging,
- detected the AprilTag,
- entered docking `CONTROLLING` state,

but the robot remained stationary.

The fix was:

```yaml
docking_server:
  ros__parameters:
    enable_stamped_cmd_vel: true
```

After this change, docking completed successfully.

Debug check:

```bash
ros2 param get /docking_server enable_stamped_cmd_vel
```

Expected:

```text
Boolean value is: True
```

---

## 17. Collision Monitor

Current Collision Monitor configuration uses LiDAR and the robot footprint.

Important values:

```text
input command:        cmd_vel_smoothed
output command:       cmd_vel
base frame:           base_footprint
odom frame:           odom
source timeout:       1.0 s
```

Current polygon mode:

```text
FootprintApproach
action_type: approach
time_before_collision: 1.2 s
minimum points: 6
```

This provides operational collision protection but is not a certified hardware safety layer.

---

## 18. Mission Layer

### 18.1 `GoToStation`

Interface:

```text
string station_name
---
bool success
string message
uint16 nav2_error_code
---
string state
float32 distance_remaining
int16 number_of_recoveries
```

Action server:

```text
/mission/go_to_station
```

Example:

```bash
ros2 action send_goal /mission/go_to_station \
  amr_interfaces/action/GoToStation \
  "{station_name: 'station_a'}" \
  --feedback
```

Current behavior:

```text
GoToStation
   |
   v
RobotExecutor
   |
   v
lookup station in stations.yaml
   |
   v
NavigateToPose
```

`RobotExecutor`:

- validates the station name,
- rejects concurrent commands,
- builds a Nav2 `NavigateToPose`,
- converts station yaw to quaternion,
- relays Nav2 feedback,
- propagates success/failure/cancellation.

**Current limitation:** `GoToStation` does not yet call Docking Server. Precision docking is currently tested directly through `DockRobot`.

### 18.2 `ExecuteMission`

Interface:

```text
string[] station_names
---
bool success
string message
uint16 completed_steps
string failed_station
---
string state
string current_station
uint16 current_step
uint16 total_steps
float32 distance_remaining
int16 number_of_recoveries
```

Action server:

```text
/mission/execute_mission
```

Example:

```bash
ros2 action send_goal /mission/execute_mission \
  amr_interfaces/action/ExecuteMission \
  "{station_names: ['station_a', 'station_b', 'home']}" \
  --feedback
```

`MissionManager` executes one `GoToStation` goal at a time and reports which station/step is active.

---

## 19. AprilTag Docking Design

### 19.1 Role of the AprilTag

The tag does not replace AMCL.

Architecture:

```text
LiDAR + AMCL:
warehouse-scale localization

AprilTag:
local station-relative precision
```

Nav2 uses the nominal dock position to reach the station area. Once the marker becomes visible, perception refines the final pose.

### 19.2 Station A marker

Current marker:

```text
Family:       AprilTag 36h11
ID:           1
Tag edge:     0.18 m
Board size:   0.225 m
```

Simulation pose:

```text
x = 1.549 m
y = -0.175 m
z = 0.002 m
yaw = -π/2
```

Station A final robot pose:

```text
x = 0.799 m
y = -0.175 m
yaw = 0
```

Therefore:

```text
tag center is 0.75 m forward of final base_footprint
```

The tag is raised 2 mm above the floor to avoid z-fighting.

The printed tag is rotated so its visual top points along the station's nominal forward direction, map `+X` for Station A.

### 19.3 Tag generation

The marker is generated using OpenCV's AprilTag 36h11 dictionary.

Texture:

```text
amr_simulation/models/apriltag_station_a/materials/textures/tag.png
```

The image contains a 1024 px marker on a 1280 px white canvas, providing a quiet zone.

Working SDF texture reference:

```xml
<albedo_map>materials/textures/tag.png</albedo_map>
```

An earlier resource-path form caused the tag plane to appear as a black rectangle. Using the relative path fixed it.

---

## 20. AprilTag Detector

Configuration:

```text
src/amr_perception/config/apriltags.yaml
```

Current parameters:

```text
family:                  36h11
default tag size:        0.18 m
pose estimation:         pnp
max hamming:             0
threads:                 2
decimate:                1.0
blur:                    0
refine:                  true
sharpening:              0.25
debug:                   false
```

Known marker:

```text
ID:      1
frame:   tag_station_a
size:    0.18 m
```

Detector input:

```text
/camera/image_raw
/camera/camera_info
```

Detector output:

```text
/apriltag/detections
tag_station_a TF
```

Launch:

```bash
ros2 launch amr_perception apriltag.launch.py
```

That launch currently starts both:

- `apriltag_ros/apriltag_node`,
- custom `dock_pose_publisher`.

---

## 21. Dock Pose Derivation

A floor tag produces a full 3D pose. Its raw roll/pitch/yaw should not be treated as robot planar yaw.

The custom node derives a horizontal station direction from the tag rotation matrix.

Current validated convention:

```text
station forward = -tag_Y
```

For tag quaternion:

```text
qx, qy, qz, qw
```

Tag +Y projected into the fixed frame is calculated from the second rotation-matrix column:

```text
tag_y_x = 2(qx*qy - qz*qw)
tag_y_y = 1 - 2(qx² + qz²)
```

Station forward:

```text
forward_x = -tag_y_x
forward_y = -tag_y_y
```

After XY normalization:

```text
dock_yaw = atan2(forward_y, forward_x)
```

Because the tag is 0.75 m ahead of the desired final robot origin:

```text
dock_x = tag_x - 0.75 * forward_x
dock_y = tag_y - 0.75 * forward_y
```

Output:

```text
geometry_msgs/PoseStamped
Topic: /detected_dock_pose
Frame: odom
```

Using `odom` is intentional because final closed-loop docking should use a locally smooth reference frame rather than directly depend on global localization corrections.

---

## 22. Perception Freshness

Current `dock_pose_publisher` parameters:

```text
tag_id:             1
tag_frame:          tag_station_a
fixed_frame:        odom
dock_offset:        0.75 m
detection_timeout:  0.5 s
timer period:       0.05 s
```

The node:

1. receives `/apriltag/detections`,
2. selects ID 1,
3. records detection time,
4. looks up `odom -> tag_station_a`,
5. derives station forward direction,
6. computes the final `base_footprint` pose,
7. publishes `/detected_dock_pose`,
8. stops using stale measurements.

Nav2's dock plugin independently uses a 1.0 s external-detection timeout.

---

## 23. Validated AprilTag Geometry

One recorded `map -> tag_station_a` measurement gave approximately:

```text
x ≈ 1.622
y ≈ -0.201
z ≈ 0.001
planar orientation representation ≈ 89.9°
```

The detected tag frame's projected `-Y` direction pointed approximately along map `+X`, confirming the intended station-forward convention.

A recorded `/detected_dock_pose` in `odom` was approximately:

```text
x = 8.39435
y = -1.37389
yaw ≈ 0.10°
```

Given Home:

```text
(-7.589, 1.2)
```

and Station A:

```text
(0.799, -0.175)
```

nominal Station A relative to the initial odometry origin was approximately:

```text
x = 8.388
y = -1.375
```

That run showed only several millimetres of position difference between the inferred local dock target and the nominal geometry, validating the tag orientation and conversion logic.

---

## 24. Nav2 Docking Server

Current dock type:

```text
opennav_docking::SimpleNonChargingDock
```

Main server parameters:

```text
controller frequency:            50 Hz
initial perception timeout:      5.0 s
dock approach timeout:           30.0 s
max retries:                     3
base frame:                      base_footprint
fixed frame:                     odom
dock backwards:                  false
prestaging tolerance:            0.50 m
enable_stamped_cmd_vel:          true
```

Plugin parameters:

```text
docking threshold:               0.05 m
staging x offset:               -0.70 m
staging yaw offset:              0
use external detection pose:     true
external detection timeout:      1.0 s
external detection translation:  0, 0
external detection rotation:     0, 0, 0
filter coefficient:              0.10
stall detection:                 false
```

Because `dock_pose_publisher` already converts raw marker pose into desired final robot pose, Nav2 must **not** apply another marker-to-dock transform.

Docking controller:

```text
k_phi:                    3.0
k_delta:                  2.0
minimum linear speed:     0.05 m/s
maximum linear speed:     0.15 m/s
maximum angular speed:    0.75 rad/s
slowdown radius:          0.25 m
collision detection:      enabled
projection time:          1.0 s
simulation step:          0.1 s
dock collision threshold: 0.0
```

`dock_collision_threshold = 0.0` is appropriate because Station A is not intended to make physical contact with a charging fixture.

---

## 25. Temporary Dock Database

Current `nav2_params.yaml` contains a temporary dock database entry:

```yaml
docks:
  - "station_a"

station_a:
  type: "load_station_dock"
  frame: "map"
  pose:
    - 0.799
    - -0.175
    - 0.0
```

This duplicates Station A's pose from `stations.yaml` and is deliberately temporary.

The next architecture step is to establish one source of truth for station and docking metadata.

---

## 26. How Docking Works From Anywhere in the Warehouse

The robot does **not** need to see the AprilTag before `DockRobot` is sent.

With:

```yaml
navigate_to_staging_pose: true
```

Nav2 performs:

```text
current robot pose
      |
      v
lookup nominal dock pose
      |
      v
compute staging pose
      |
      v
ordinary Nav2 navigation
      |
      v
arrive near Station A
      |
      v
camera sees AprilTag
      |
      v
initial perception
      |
      v
/detected_dock_pose
      |
      v
precision docking controller
      |
      v
final pose
```

For Station A:

```text
final pose:            (0.799, -0.175, 0)
staging x offset:      -0.70 m
nominal staging pose:  (0.099, -0.175, 0)
tag pose:              (1.549, -0.175)
```

At staging, the tag is about 1.45 m ahead of `base_footprint`, roughly 1.0 m ahead of the front-mounted camera.

---

## 27. Running the Current System

### 27.1 Build

```bash
cd ~/Projects/warehouse_amr
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 27.2 Start simulation + navigation + missions

```bash
ros2 launch amr_bringup simulation_navigation.launch.py
```

This starts:

- robot description,
- Gazebo warehouse,
- ROS/Gazebo bridge,
- robot spawn at Home,
- controller manager,
- joint state broadcaster,
- differential-drive controller,
- EKF,
- saved map,
- AMCL,
- Nav2,
- RViz,
- RobotExecutor,
- MissionManager.

### 27.3 Start docking perception

Currently separate:

```bash
ros2 launch amr_perception apriltag.launch.py
```

Later this should be integrated or lifecycle-controlled so camera/perception becomes demand-driven.

---

## 28. Navigation Test Commands

Station A:

```bash
ros2 action send_goal /mission/go_to_station \
  amr_interfaces/action/GoToStation \
  "{station_name: 'station_a'}" \
  --feedback
```

Station B:

```bash
ros2 action send_goal /mission/go_to_station \
  amr_interfaces/action/GoToStation \
  "{station_name: 'station_b'}" \
  --feedback
```

Home:

```bash
ros2 action send_goal /mission/go_to_station \
  amr_interfaces/action/GoToStation \
  "{station_name: 'home'}" \
  --feedback
```

These currently use ordinary `NavigateToPose` and therefore do not perform AprilTag final alignment.

---

## 29. Direct Docking Test

Check actions:

```bash
ros2 action list | grep dock
```

Expected:

```text
/dock_robot
/undock_robot
```

Check lifecycle:

```bash
ros2 lifecycle get /docking_server
```

Expected:

```text
active [3]
```

Check local costmap:

```bash
ros2 lifecycle get /local_costmap/local_costmap
```

Expected:

```text
active [3]
```

Send Station A docking goal:

```bash
ros2 action send_goal /dock_robot \
  nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: 'station_a', navigate_to_staging_pose: true}" \
  --feedback
```

Successful result:

```text
success: true
error_code: 0
num_retries: 0
```

---

## 30. Docking Feedback States

Observed states include:

```text
1  NAV_TO_STAGING_POSE
2  INITIAL_PERCEPTION
3  CONTROLLING
```

For the current non-charging dock, the server can remain in state 3 until the final positional criterion is satisfied and then return `SUCCEEDED`.

Repeated `state: 3` is not itself a failure. It means the docking controller is running.

If state 3 persists while the robot is stationary, first verify the stamped command setting and inspect `/cmd_vel`.

---

## 31. Perception Debug Commands

Camera rates:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/camera_info
```

View image:

```bash
ros2 run rqt_image_view rqt_image_view
```

AprilTag detections:

```bash
ros2 topic echo /apriltag/detections
```

Camera-to-tag transform:

```bash
ros2 run tf2_ros tf2_echo \
  camera_optical_frame \
  tag_station_a
```

Map-to-tag transform:

```bash
ros2 run tf2_ros tf2_echo \
  map \
  tag_station_a
```

Derived dock pose:

```bash
ros2 topic echo /detected_dock_pose
```

Velocity commands:

```bash
ros2 topic echo /cmd_vel
```

---

## 32. Navigation Debug Commands

Global pose:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Local odometry pose:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

LiDAR rate:

```bash
ros2 topic hz /scan
```

Filtered odometry:

```bash
ros2 topic echo /odom
```

Controllers:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster
diff_drive_controller
```

Lifecycle checks:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /local_costmap/local_costmap
ros2 lifecycle get /global_costmap/global_costmap
ros2 lifecycle get /docking_server
```

---

## 33. Important Debugging Lessons

### 33.1 Camera rate follows simulation real-time factor

If Gazebo runs at 0.7 RTF, a 10 Hz simulated sensor may appear around 7 Hz in wall-clock measurements. Do not diagnose this as dropped sensor data without checking simulation time.

### 33.2 High-resolution camera can overload simulation

The earlier 1456×1088 @ 30 Hz camera configuration caused severe Gazebo load. Keep the current 640×480 @ 10 Hz baseline until there is a reason to increase it.

### 33.3 Empty AprilTag arrays are valid

```yaml
detections: []
```

means the detector is alive and receiving images but currently sees no valid marker.

### 33.4 Gazebo spectator view is not camera view

A tag visible in the Gazebo GUI may still be outside `/camera/image_raw`. Always inspect the actual robot camera image when debugging detection.

### 33.5 Do not use raw floor-tag RPY as station yaw

The raw 3D orientation contains camera tilt and floor-plane geometry. Use an in-plane direction vector projected to XY, as implemented in `dock_pose_publisher`.

### 33.6 Texture paths matter

The AprilTag initially rendered as a black rectangle. The working SDF uses a model-relative texture path.

### 33.7 One owner per TF transform

Current ownership:

```text
AMCL / SLAM Toolbox: map -> odom
EKF:                 odom -> base_footprint
robot_state_pub:     base_footprint -> sensors/links
```

The diff-drive controller has `enable_odom_tf: false` to avoid conflict.

### 33.8 Docking must use TwistStamped

The docking server must have:

```yaml
enable_stamped_cmd_vel: true
```

This was a real failure mode: the server reached `CONTROLLING`, but the robot did not move until this was corrected.

### 33.9 Preserve Nav2 YAML structure

A previous manual docking edit accidentally removed the costmap sections. The result was a dead local costmap even though docking itself was unrelated.

Before launching after large YAML edits:

```bash
python3 - <<'PY'
import yaml

path = "src/amr_navigation/config/nav2_params.yaml"

with open(path) as f:
    data = yaml.safe_load(f)

for key in [
    "controller_server",
    "local_costmap",
    "global_costmap",
    "planner_server",
    "collision_monitor",
    "docking_server",
]:
    print(key, key in data)
PY
```

---

## 34. ROS 2 Network Isolation During Development

ROS 2 DDS discovery can cross between machines on the same LAN/Wi-Fi. Two unrelated developers can accidentally see each other's topics, TF and robots.

For independent development sessions, use different domains:

```bash
export ROS_DOMAIN_ID=21
```

Another developer can use, for example:

```bash
export ROS_DOMAIN_ID=22
```

For a future intentional fleet, robots should use deliberate namespaces, IDs and fleet communication rather than accidental DDS separation.

---

## 35. Current Known Limitations

1. Only Station A has a simulated AprilTag and verified precision docking.
2. `dock_pose_publisher` is configured for one tag/station.
3. Station A coordinates are duplicated in `stations.yaml` and Nav2 docking parameters.
4. `RobotExecutor` still sends `NavigateToPose`; it does not yet invoke `DockRobot`.
5. `MissionManager` therefore cannot yet request precision docking through its normal station workflow.
6. Perception is launched manually rather than around docking demand.
7. Camera activation is not yet lifecycle-managed by mission/docking state.
8. Home is not yet modeled as a real charging dock.
9. Station B has no precision marker yet.
10. No real camera driver has been integrated.
11. No real PLC / BLDC drive interface has been implemented.
12. No hardware safety PLC, safety scanner or STO chain has been implemented.
13. No multi-robot namespace/fleet system has been implemented.
14. Payload dynamics are not yet modeled.
15. Gazebo physics probably uses a smaller timestep than necessary.
16. Several package manifests need dependency/metadata cleanup.
17. Automated integration tests are still minimal.

---

## 36. Immediate Next Architecture Step

The next task is station/docking generalization without changing the successful Station A controller behavior.

Current duplication:

```text
stations.yaml
    station_a coordinates

nav2_params.yaml
    station_a coordinates again
```

The goal is one authoritative station definition containing position and docking metadata.

A likely future schema is:

```yaml
frame_id: map

stations:

  home:
    x: -7.589
    y: 1.2
    yaw: 0.0
    precision_dock: true
    dock_type: charging
    tag_id: 3

  station_a:
    x: 0.799
    y: -0.175
    yaw: 0.0
    precision_dock: true
    dock_type: load_station
    tag_id: 1

  station_b:
    x: 8.716
    y: 2.705
    yaw: 0.0
    precision_dock: true
    dock_type: load_station
    tag_id: 2
```

The exact final schema is not yet fixed; this shows the intended direction.

From one source of truth, bringup should provide:

- nominal dock poses to Docking Server,
- tag IDs to perception,
- station metadata to mission logic.

---

## 37. Target Mission Flow

```text
application / operator / fleet manager
                |
                v
      ExecuteMission or GoToStation
                |
                v
          RobotExecutor
                |
        +-------+-------+
        |               |
 normal station      dock station
        |               |
NavigateToPose       DockRobot
                        |
                        v
                  staging Nav2
                        |
                        v
                  AprilTag refine
                        |
                        v
                  final docking
```

External callers should continue using the simple station-level API:

```bash
ros2 action send_goal /mission/go_to_station \
  amr_interfaces/action/GoToStation \
  "{station_name: 'station_a'}" \
  --feedback
```

The caller should not need to know whether that station uses plain Nav2, AprilTag docking or future charging logic.

---

## 38. Recommended Docking Generalization Order

```text
1. Keep current Station A docking unchanged as regression baseline.
2. Remove duplicate station coordinates.
3. Add Station B tag and metadata.
4. Generalize dock-pose perception to selected tag/station.
5. Integrate DockRobot into RobotExecutor.
6. Integrate perception activation with docking state.
7. Add Home as a dock.
8. Add charging feedback only when real hardware exists.
```

Use `SimpleNonChargingDock` for simulated loading stations. A future charging dock should only use charging-success logic when real contact/charging feedback exists.

---

## 39. Multi-Robot / Fleet Roadmap

Before adding fleet logic, each robot should have:

- reliable localization,
- reliable station navigation,
- reliable precision docking,
- deterministic mission cancellation/failure behavior,
- clean namespace handling.

Future multi-robot work will require namespaced interfaces such as:

```text
/robot_1/cmd_vel
/robot_1/odom
/robot_1/scan
/robot_1/tf
/robot_1/mission/...
```

A custom fleet manager can first be used to learn and validate:

- task allocation,
- robot state tracking,
- station locking,
- route/traffic conflicts,
- mission queues,
- failure recovery.

Open-RMF can then be evaluated against the custom implementation.

---

## 40. Sim-to-Real Migration Strategy

High-level ROS software should not care whether wheel control comes from Gazebo or real hardware.

Desired abstraction:

```text
Nav2 / mission logic
        |
        v
robot drive interface
        |
    +---+---+
    |       |
 Gazebo    PLC/drive bridge
            |
            v
         real motors
```

Likewise, maintain stable ROS sensor interfaces:

```text
Simulation LiDAR  ---> /scan
Real LiDAR        ---> /scan

Simulation IMU    ---> /imu/data_raw
Real IMU          ---> /imu/data_raw

Gazebo camera     ---> /camera/image_raw
Real camera       ---> /camera/image_raw
```

This keeps the navigation/perception stack independent of the physical I/O implementation.

---

## 41. Real-Hardware Direction

Current reference camera concept is a Raspberry Pi Global Shutter Camera-class device because fiducial docking benefits from global shutter under motion.

Potential ROS compute:

```text
Raspberry Pi 5
```

or industrial x86 if processing, reliability or fleet requirements justify it.

The project also plans Siemens S7-1500-class PLC hardware for deterministic machine-level control.

Final hardware choices should be validated against:

- actual BLDC drives,
- encoder interfaces,
- safety architecture,
- camera driver availability,
- PLC communication latency,
- required cycle time,
- environmental/EMC requirements.

---

## 42. Verified Baseline Values

These values currently form the known-good baseline and should not be changed casually:

```text
wheel radius                     0.10 m
wheel separation                 0.69 m

EKF owns                         odom -> base_footprint
AMCL owns                        map -> odom

LiDAR                            720 samples, 10 Hz
camera                           640×480, 10 Hz
camera pitch                     25° downward

Nav2 controller                  Regulated Pure Pursuit
Nav2 planner                     SmacPlanner2D

local costmap                    7×7 m, 0.05 m resolution
inflation radius                 0.70 m

Station A                        (0.799, -0.175, 0)
Station A tag                    (1.549, -0.175), ID 1
tag-to-final-base offset         0.75 m

dock staging offset              -0.70 m
dock threshold                   0.05 m
dock speed                       0.05–0.15 m/s
Docking Server TwistStamped      enabled
```

Changes should be driven by physical requirements or measured behavior, followed by focused regression testing.

---

## 43. Minimal Regression Test Checklist

After a major configuration or architecture change, verify in this order.

### Robot/control

```bash
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic hz /imu/data_raw
ros2 topic hz /scan
```

### TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

### Nav2

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /local_costmap/local_costmap
ros2 lifecycle get /global_costmap/global_costmap
```

### Normal navigation

Send `GoToStation` to a known station and confirm path following.

### Perception

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /apriltag/detections
ros2 topic echo /detected_dock_pose
```

### Docking

```bash
ros2 lifecycle get /docking_server
ros2 param get /docking_server enable_stamped_cmd_vel
```

Then:

```bash
ros2 action send_goal /dock_robot \
  nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: 'station_a', navigate_to_staging_pose: true}" \
  --feedback
```

Expected:

```text
success: true
error_code: 0
```

---

## 44. Git Workflow

Use Git checkpoints before architecture changes.

Recommended workflow:

```bash
git status
git diff
git add .
git commit -m "descriptive milestone"
git push
```

Before a risky configuration edit:

```bash
git status
```

Restore one file without discarding unrelated work:

```bash
git restore --source=HEAD -- path/to/file
```

Do not restore the entire repository casually when unrelated uncommitted work exists.

---

## 45. Project Design Principles

1. **One component owns each TF transform.**
2. **Global localization and local precision docking solve different problems.**
3. **Station names are the application interface; coordinates are configuration.**
4. **Perception refines station pose; it does not replace global localization.**
5. **Simulation and real hardware should expose equivalent ROS interfaces.**
6. **Mission logic should not contain controller details.**
7. **Safety must not depend exclusively on ROS application software.**
8. **Avoid duplicate configuration sources.**
9. **Do not tune around an architectural error.**
10. **Preserve known-good checkpoints before major changes.**
11. **Test the subsystem that changed rather than retesting everything after every small edit.**
12. **Use measured geometry and sensor constraints instead of arbitrary tuning values.**

---

## 46. Current Project State — One-Page Summary

```text
PLATFORM
-------
Differential-drive warehouse AMR
1.00 m × 0.60 m collision envelope
2 powered wheels + 4 passive supports
ROS 2 Jazzy
Gazebo Harmonic

LOCAL CONTROL
-------------
ros2_control
DiffDriveController
wheel radius 0.10 m
wheel separation 0.69 m

STATE ESTIMATION
----------------
wheel odometry + IMU
robot_localization EKF
odom -> base_footprint

GLOBAL LOCALIZATION
-------------------
saved map + AMCL
map -> odom

MAPPING
-------
SLAM Toolbox
2D LiDAR

NAVIGATION
----------
Nav2
SmacPlanner2D
Regulated Pure Pursuit
local + global obstacle costmaps
velocity smoother
collision monitor

MISSION LAYER
-------------
stations.yaml
RobotExecutor
MissionManager
GoToStation
ExecuteMission

DOCKING
-------
front/downward 640×480 simulated camera
AprilTag 36h11
Station A tag ID 1
tag edge 0.18 m
custom tag -> dock pose conversion
Nav2 SimpleNonChargingDock
precision speed 0.05–0.15 m/s
verified successful docking
zero retries in successful validation run

CURRENT TEMPORARY ITEMS
-----------------------
Station A duplicated in Nav2 docking database
perception manually launched
dock_pose_publisher configured for Station A only
GoToStation still uses NavigateToPose, not DockRobot

NEXT WORK
---------
single source of truth for stations/docks
generalize tag/dock configuration
integrate DockRobot into RobotExecutor
add Station B and Home docking
on-demand perception
then multi-robot/fleet work
```

---

## 47. Recommended Starting Procedure for a New Developer

A developer joining the project should reproduce the known-good baseline before modifying controllers or architecture.

```text
1. Clone the repository.
2. Install ROS 2 Jazzy and required dependencies.
3. Build with colcon.
4. Launch simulation_navigation.
5. Confirm /scan, /imu/data_raw and /odom.
6. Confirm map -> odom -> base_footprint TF.
7. Send GoToStation to station_a.
8. Start apriltag.launch.py.
9. Confirm camera image and tag detection.
10. Confirm /detected_dock_pose.
11. Send DockRobot to station_a.
12. Confirm SUCCEEDED with error_code 0.
13. Only then begin new development.
```

Main launch:

```bash
cd ~/Projects/warehouse_amr
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch amr_bringup simulation_navigation.launch.py
```

Perception in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/warehouse_amr/install/setup.bash
ros2 launch amr_perception apriltag.launch.py
```

Verified docking command:

```bash
ros2 action send_goal /dock_robot \
  nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: 'station_a', navigate_to_staging_pose: true}" \
  --feedback
```

If that returns:

```text
success: true
error_code: 0
```

the current navigation/perception/docking baseline is functioning.

---

## 48. End-State Direction

```text
                FLEET / OPERATOR
                      |
                      v
                mission request
                      |
                      v
               MissionManager
                      |
                      v
                RobotExecutor
                      |
              station metadata
                      |
           +----------+----------+
           |                     |
           v                     v
     NavigateToPose          DockRobot
                                 |
                           precision sensing
                                 |
                           final alignment
                                 |
                                 v
                          drive abstraction
                                 |
                  +--------------+--------------+
                  |                             |
               Gazebo                    PLC + motor drives
```

The project is already beyond the basic stage of getting a robot to move with Nav2. The current engineering focus is converting the successful Station A docking proof into a clean reusable station architecture before moving to multi-robot fleet behavior and real-hardware integration.
