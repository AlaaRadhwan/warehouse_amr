# Warehouse AMR — Comprehensive Project Documentation

**Project:** `warehouse_amr`  
**Repository:** `AlaaRadhwan/warehouse_amr`  
**Platform:** ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04  
**Current checkpoint date:** 2026-10-18  
**Latest confirmed `main` commit:** `fe42d1aa2037103213093c444b8b1fa74708eaf3`  
**Commit message:** `Integrate DockRobot missions and test staging navigation behavior`

---

# 1. Purpose of This Document

This document records the current engineering state of the warehouse AMR project, including:

- system architecture;
- robot model and simulation assumptions;
- ROS 2 package structure;
- control, localization, planning, safety and mission architecture;
- station and docking design;
- AprilTag precision-docking implementation;
- tests that have been run and what they proved;
- failures and debugging investigations;
- conclusions that were discarded or revised;
- the current unresolved navigation issue;
- the new long-term architecture and implementation plan.

The main goal is to preserve the reasoning behind the design, not only the final code.

---

# 2. Project Goal

The target system is a small industrial warehouse AMR capable of:

- transporting approximately **50 kg payload**;
- differential-drive navigation;
- autonomous localization and navigation;
- repeatable parking and station arrival;
- precise station alignment where required;
- eventual multi-robot operation;
- later integration with a Siemens PLC / industrial control layer;
- operation in a real warehouse, where deterministic motion, resource coordination and independent functional safety matter.

The current robot concept is approximately:

- **length:** 1.0 m;
- **width:** 0.60 m;
- **differential drive:** 2 powered wheels;
- **support:** 4 passive caster/support wheels;
- **nominal base mass in simulation:** approximately 50 kg;
- **payload target:** approximately 50 kg.

The simulation is not intended to remain a research-only robot. The architecture is being developed toward an industrial system.

---

# 3. High-Level System Architecture

The intended final architecture separates high-level autonomy from deterministic machine control.

```text
                Fleet / Mission Layer
                        |
                        v
                ROS 2 Autonomy IPC
      ----------------------------------------
      | Mission logic                       |
      | Route / traffic interface           |
      | Global planning                     |
      | Local planning / control            |
      | Localization                        |
      | Perception                          |
      | Station / docking logic             |
      ----------------------------------------
                        |
                        | desired motion / commands
                        v
                 PLC / Drive Layer
      ----------------------------------------
      | deterministic machine control       |
      | watchdogs                           |
      | drive interface                     |
      | process interlocks                  |
      ----------------------------------------
                        |
                        v
                    Motor Drives

Independent safety path:
Safety lidar / E-stops / safety PLC -> STO / safe motion
```

Important principle:

> ROS / Nav2 operational collision avoidance is **not** treated as the certified safety layer.

The long-term architecture therefore keeps industrial safety independent from the autonomy software.

---

# 4. Development Environment

Current development environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `gz sim` 8.x
- workspace: `~/Projects/warehouse_amr`

Typical shell setup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/warehouse_amr/install/setup.bash
```

Main packages currently include:

```text
amr_description
amr_simulation
amr_control
amr_localization
amr_navigation
amr_interfaces
amr_missions
amr_bringup
amr_perception
```

---

# 5. ROS Package Responsibilities

## 5.1 `amr_description`

Responsible for:

- URDF / Xacro;
- robot geometry;
- collision geometry;
- wheel geometry;
- sensor frames;
- simulated actuators and sensors.

The robot uses REP-style frames with:

```text
x = forward
y = left
z = up
```

Important frames:

```text
base_footprint
base_link
left / right wheel links
imu_link
lidar_link
camera frames
camera_optical_frame
```

## 5.2 `amr_simulation`

Responsible for:

- Gazebo world;
- robot spawning;
- Gazebo / ROS bridges;
- warehouse environment;
- simulated AprilTag station target.

## 5.3 `amr_control`

Responsible for `ros2_control`.

Current drive:

```text
diff_drive_controller
```

Current values:

```text
wheel_separation = 0.69 m
wheel_radius     = 0.10 m
controller rate  = 100 Hz
odom publish     = 50 Hz
```

Velocity limits:

```text
linear:
    max = +1.0 m/s
    min = -1.0 m/s
    max acceleration = 0.8 m/s²

angular:
    max = +1.2 rad/s
    min = -1.2 rad/s
    max acceleration = 1.5 rad/s²
```

The diff-drive controller does **not** own `odom -> base_footprint` TF:

```yaml
enable_odom_tf: false
```

The EKF owns that transform.

---

# 6. Robot Geometry

Current principal dimensions:

```text
base_length = 1.00 m
base_width  = 0.60 m
base_height = 0.20 m
```

Navigation footprint currently uses approximately:

```text
x = ±0.50 m
y = ±0.37 m
```

This is intentionally conservative relative to the visual body.

Approximate wheel geometry:

```text
wheel radius      = 0.10 m
wheel width       = 0.05 m
wheel separation  = 0.69 m
```

Four passive supports/casters are used in simulation.

The camera is mounted near the front of the robot and pitched downward for floor AprilTag detection.

---

# 7. Sensors and ROS Interfaces

The main simulated ROS interfaces are:

```text
/cmd_vel
/scan
/imu/data_raw
/odom
/joint_states
/tf
/tf_static

/camera/image_raw
/camera/camera_info
```

The autonomy contract intentionally uses standard ROS interfaces so the simulated robot can later be replaced by real hardware while keeping most of Nav2 unchanged.

---

# 8. Localization Architecture

Current localization uses:

```text
wheel odometry
    +
IMU yaw rate
    ->
robot_localization EKF
    ->
odom -> base_footprint
```

Saved-map localization uses AMCL:

```text
map -> odom
```

TF ownership is therefore:

```text
AMCL / SLAM       : map  -> odom
EKF               : odom -> base_footprint
robot_state_pub   : base -> sensor / robot links
```

This avoids duplicate TF publishers.

---

# 9. Current Nav2 Architecture

Current global planner:

```text
nav2_smac_planner::SmacPlanner2D
```

Current controller:

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

Current main goal checker:

```text
nav2_controller::SimpleGoalChecker
```

Current important goal settings:

```text
xy_goal_tolerance  = 0.10 m
yaw_goal_tolerance = 0.05 rad
stateful            = true
```

RPP currently uses:

```text
desired_linear_vel            = 0.8 m/s
use_rotate_to_heading         = true
rotate_to_heading_angular_vel = 0.45 rad/s
rotate_to_heading_min_angle   = 0.6 rad
max_angular_accel             = 1.0 rad/s²
allow_reversing               = false
stateful                      = true
```

Local and global costmaps use the rectangular robot footprint rather than a point approximation.

---

# 10. Collision Monitoring

Nav2 Collision Monitor is configured as an **operational collision-prevention layer**.

It consumes:

```text
scan
local footprint
smoothed command velocity
```

and can stop / modify commands before they reach the drivetrain.

Important design conclusion:

> Collision Monitor is useful for autonomy robustness but is not considered a functional-safety replacement for safety lidar, safety PLC logic or STO.

---

# 11. Mission Architecture

The mission layer is intentionally separated from Nav2.

Two custom actions currently exist.

## 11.1 `GoToStation`

```text
Goal:
    string station_name

Result:
    bool success
    string message
    uint16 nav2_error_code

Feedback:
    string state
    float32 distance_remaining
    int16 number_of_recoveries
```

## 11.2 `ExecuteMission`

```text
Goal:
    string[] station_names

Result:
    bool success
    string message
    uint16 completed_steps
    string failed_station

Feedback:
    string state
    string current_station
    uint16 current_step
    uint16 total_steps
    float32 distance_remaining
    int16 number_of_recoveries
```

---

# 12. `RobotExecutor`

`RobotExecutor` provides:

```text
/mission/go_to_station
```

It is an action server to the application layer and an action client to Nav2.

Current downstream actions:

```text
NavigateToPose
DockRobot
```

Routing logic:

```text
station
  |
  +-- docking.enabled == true
  |       |
  |       -> DockRobot
  |
  +-- otherwise
          |
          -> NavigateToPose
```

Only one downstream goal is considered active at a time through:

```python
self.active_downstream_goal
```

This allows mission cancellation to propagate to normal navigation or docking.

### Important implementation lessons

`RobotExecutor` uses:

```text
ReentrantCallbackGroup
MultiThreadedExecutor
```

Important concurrency conclusions:

- an rclpy coroutine awaiting a Future yields control to the executor;
- it should not be described as simply blocking a worker thread;
- callback groups determine whether callbacks may overlap;
- worker-to-callback mapping should not be assumed;
- `wait_for_server(timeout_sec=...)` itself is synchronous.

### Known future improvement

The current:

```python
self.busy
```

gate is application-level state, not a formal synchronization primitive.

With a reentrant callback group and multi-threaded executor there is a theoretical goal-admission race. It has not been identified as the current navigation problem, so it has deliberately not been refactored yet.

---

# 13. `MissionManager`

`MissionManager` provides:

```text
/mission/execute_mission
```

It executes station sequences through `GoToStation`.

Its purpose is to keep mission sequencing independent from:

- Nav2 details;
- docking details;
- controller implementation.

This becomes useful later for loading, unloading, PLC handshakes, station locks and fleet reservations.

---

# 14. Station Configuration

The project moved toward a **single source of truth** for stations.

Concept:

```yaml
frame_id: map

stations:

  home:
    x: ...
    y: ...
    yaw: ...

  station_a:
    x: ...
    y: ...
    yaw: ...

    docking:
      enabled: true
      type: ...
      tag_id: ...
      tag_frame: ...
      tag_size: ...
      dock_offset: ...

  station_b:
    x: ...
    y: ...
    yaw: ...
```

Key decision:

> Station instance geometry should not be duplicated manually inside Nav2 docking parameters.

Instead, bringup reads `stations.yaml` and generates the runtime docking database.

---

# 15. Current Known Stations

```text
home
    x = -7.589
    y =  1.2
    yaw = 0

station_a
    x =  0.799
    y = -0.175
    yaw = 0
    precision docking enabled

station_b
    x = 8.716
    y = 2.705
    yaw = 0
```

Station A currently uses AprilTag ID 1.

---

# 16. Nav2 Docking Server Integration

Current docking action:

```text
nav2_msgs/action/DockRobot
```

The mission code uses:

```text
use_dock_id = true
dock_id = station_name
navigate_to_staging_pose = true
```

This means the Docking Server is responsible for:

1. loading the dock instance;
2. computing the staging pose;
3. calling Nav2 to reach staging;
4. performing initial perception;
5. using refined dock perception;
6. executing the docking control law.

This is better than duplicating staging-navigation logic in `RobotExecutor`.

---

# 17. Dock Database Generation

At bringup:

```text
stations.yaml
    |
    v
simulation_navigation.launch.py
    |
    v
temporary Nav2 docking database YAML
    |
    v
docking_server.dock_database
```

The dock instance uses:

```text
station name
station pose
station frame
dock type
```

Therefore station geometry remains in `stations.yaml`, while the docking server receives a native Nav2 dock database.

---

# 18. Station A Docking Geometry

Station A final robot pose:

```text
x   = 0.799
y   = -0.175
yaw = 0
```

The AprilTag is located approximately:

```text
0.75 m ahead of the final base_footprint pose
```

The docking plugin staging pose is approximately:

```text
0.70 m behind the final dock pose
```

Current plugin parameters:

```yaml
staging_x_offset: -0.70
staging_yaw_offset: 0.0
```

Concept:

```text
AprilTag
    |
~0.75 m
    |
final robot pose
    |
~0.70 m
    |
staging pose
```

---

# 19. AprilTag Perception

Current AprilTag configuration:

```text
family        = tag36h11
tag ID        = 1
physical size = 0.18 m
pose method   = PnP
max_hamming   = 0
```

Detection input:

```text
/camera/image_raw
/camera/camera_info
```

Outputs include:

```text
/apriltag/detections
tag_station_a
```

---

# 20. `dock_pose_publisher`

The custom node converts the observed AprilTag frame into the desired final robot pose.

It publishes:

```text
/detected_dock_pose
```

The node:

1. finds the required tag ID;
2. waits for its TF;
3. transforms the tag into the fixed frame;
4. extracts station-forward direction;
5. projects the direction into the horizontal plane;
6. moves backward from the tag by the configured dock offset;
7. publishes the desired `base_footprint` target pose.

Important geometric result:

```text
station_forward = -tag_Y
```

for the current floor-mounted tag orientation.

---

# 21. AprilTag Timing Design

A detection message may arrive slightly before the matching TF becomes available.

The implementation therefore uses:

```text
detection callback
    |
    -> mark detection pending

50 ms timer
    |
    -> try TF lookup
    |
    -> retry briefly if TF is not ready
```

This avoids an image/TF timing race.

The output pose uses the TF measurement timestamp rather than pretending an older observation is current.

---

# 22. Docking Server Parameters

Important values:

```text
controller_frequency        = 50 Hz
initial_perception_timeout  = 5 s
dock_approach_timeout       = 30 s
max_retries                 = 3

base_frame  = base_footprint
fixed_frame = odom

dock_backwards = false
dock_prestaging_tolerance = 0.50 m
```

Precision controller:

```text
k_phi   = 3.0
k_delta = 2.0

v_linear_min  = 0.05 m/s
v_linear_max  = 0.15 m/s
v_angular_max = 0.75 rad/s

slowdown_radius = 0.25 m
```

Station A is a non-contact docking target:

```text
use_stall_detection = false
dock_collision_threshold = 0.0
```

---

# 23. Important Docking Bug Found and Fixed

An earlier docking attempt reached the docking controller state but did not move correctly.

Root cause:

```text
Docking Server velocity output format
        !=
rest of command chain expectation
```

The navigation chain expected stamped velocity commands.

Fix:

```yaml
docking_server:
  ros__parameters:
    enable_stamped_cmd_vel: true
```

After this change, direct DockRobot testing succeeded.

This was an integration-interface bug, not a docking-gain problem.

---

# 24. Direct DockRobot Validation

The direct action test was validated:

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{use_dock_id: true, dock_id: 'station_a', navigate_to_staging_pose: true}" --feedback
```

Observed result:

```text
successful navigation to staging
successful initial dock detection
precision approach
successful docking
error_code = 0
num_retries = 0
```

This demonstrated that:

- dock database generation works;
- dock ID resolution works;
- staging navigation works;
- AprilTag refinement works;
- the docking controller works;
- tag-to-dock pose conversion is consistent.

---

# 25. High-Level Mission Docking Validation

After DockRobot integration into `RobotExecutor`:

```text
Home -> Station A
```

successfully executed:

```text
GoToStation
    ->
RobotExecutor detects docking.enabled
    ->
DockRobot
    ->
Nav2 staging navigation
    ->
AprilTag perception
    ->
precision docking
    ->
GoToStation success
```

Docking itself was precise and repeatable in the successful trials.

---

# 26. Normal Navigation Regression

Station B currently does not require precision docking.

Test:

```text
Station A -> Station B
```

used normal `NavigateToPose` and completed successfully.

This proved DockRobot integration did not break normal station navigation.

---

# 27. Terminal-Heading Oscillation Problem

The main unresolved navigation issue appeared during:

```text
Station B -> Station A
```

and later also during ordinary navigation such as:

```text
Station A -> Home
```

The second observation proved this is **not fundamentally an AprilTag problem**.

Observed behavior has included:

- smooth approach to target region;
- correct beginning of final heading correction;
- small left/right heading corrections;
- reversal of rotation direction;
- approximately 180° rotation away from the desired orientation;
- approximately 180° rotation back;
- eventual yaw acceptance;
- normal continuation afterward.

The behavior is intermittent:

```text
the same mission may be clean once
and oscillatory on another run
```

That is unacceptable for the intended real warehouse use.

---

# 28. What the Captured Logs Proved

A captured B -> A run showed:

```text
DockRobot requested
    ->
BT Navigator navigates from B to staging
    ->
controller follows navigation goal
    ->
"Reached the goal!"
    ->
"Successful navigation to staging pose"
    ->
"Successful initial dock detection"
    ->
successful precision docking
```

There were no matching:

```text
collision failure
failed progress
controller patience exceeded
Spin recovery
```

messages during that navigation.

Therefore an earlier hypothesis was rejected:

> The large rotations were **not primarily the Nav2 Behavior Tree Spin recovery**.

The staging navigation itself was eventually considered successful.

---

# 29. AprilTag Is Not Causing the Rotation

The robot may visually see the AprilTag before staging is complete.

The Docking Server sequence is:

```text
navigate to staging
    ->
NavigateToPose success
    ->
initial perception
    ->
precision docking control
```

Therefore an early tag sighting does not transfer control to the docking controller.

Furthermore, similar oscillation on A -> Home excludes AprilTag control as the root cause.

---

# 30. Replanning Hypothesis and Experiment

The default Nav2 `NavigateToPose` behavior tree replans periodically.

Logs showed repeated:

```text
Passing new path to controller.
```

messages.

Because RPP has internal state for final-heading handling, periodic plan replacement was considered a possible contributor.

A special docking staging BT was tested:

```text
navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml
```

The normal Nav2 BT remained:

```text
navigate_to_pose_w_replanning_and_recovery.xml
```

The configuration split was verified at runtime.

### Result

The approach trajectory changed somewhat, including more angled approaches from B to A.

However:

> the terminal oscillation still occurred.

Conclusion:

> periodic replanning may influence the exact trajectory, but it is **not the root cause**.

The special docking-BT experiment should not be treated as the final architecture.

---

# 31. Current Root-Level Diagnosis

The issue is now treated as a general planning / terminal-pose problem.

Current combination:

```text
SmacPlanner2D
        +
RPP
        +
strict x / y / yaw goals
```

has a structural weakness for orientation-sensitive destinations.

SmacPlanner2D can produce a valid XY path whose arrival direction is inconsistent with the requested final yaw.

Example:

```text
robot travels toward goal facing -X
                  |
                  v
              reaches XY

requested final orientation = +X
```

The local controller is then left to resolve a very large terminal yaw error, potentially near 180°.

That is the condition in which intermittent heading reversals have been observed.

---

# 32. Why This Is Not Only a Docking Problem

An important design correction was made during the investigation.

It would be too narrow to use a special planner only for AprilTag stations.

Real warehouse destinations where heading matters include:

```text
parking bays
home positions
waiting slots
conveyor interfaces
loading points
charging stations
pallet pickup positions
inspection stations
```

A parking slot may need a deterministic heading to guarantee:

- dense packing;
- predictable footprint occupancy;
- aisle clearance;
- predictable departure;
- sensor visibility;
- connector geometry;
- fleet coordination.

Therefore:

> final orientation is a general destination constraint, not a docking special case.

---

# 33. Revised Long-Term Destination Model

Meaningful warehouse destinations should evolve into **semantic resources**.

Conceptually:

```yaml
park_03:

  pose:
    x: ...
    y: ...
    yaw: ...

  tolerance:
    xy: ...
    yaw: ...

  resource:
    id: parking_03
    exclusive: true

  approach:
    route_node: parking_entry
    allowed_direction: forward

  precision:
    type: none
```

An AprilTag station differs mainly in the final precision operation:

```yaml
precision:
  type: apriltag_docking
```

The navigation goal remains a meaningful pose whether or not a marker exists.

---

# 34. New Primary Planner Direction: Smac State Lattice

The next major architecture experiment is:

```text
nav2_smac_planner::SmacPlannerLattice
```

for pose-constrained warehouse navigation.

The AMR is:

```text
differential drive
rectangular
non-trivial footprint
orientation-sensitive at important destinations
```

A State Lattice planner reasons over a pose state:

```text
x
y
heading
```

rather than treating heading mainly as a terminal correction.

Desired result:

```text
start pose
    ->
feasible sequence of x / y / yaw states
    ->
approach destination with meaningful orientation
    ->
small terminal correction only
```

instead of:

```text
2D path to XY
    ->
large final spin
```

---

# 35. Controller Strategy: Keep RPP First

A previous discussion considered moving directly to:

```text
State Lattice + MPPI
```

That is no longer the recommended immediate step.

The correct experimental sequence is:

```text
change planner first
keep controller unchanged
```

Initial next baseline:

```text
Smac State Lattice
        +
current RPP
```

Reasons:

- RPP is a legitimate path-following controller;
- it should operate under better conditions if the path already contains meaningful orientation;
- changing planner and controller simultaneously would prevent us from identifying which subsystem solved the problem.

Only if RPP remains unacceptable after proper lattice planning should other controllers be benchmarked.

---

# 36. Controller Candidates for Later Evaluation

If needed after State Lattice:

## MPPI

Potential strengths:

- predictive sampling;
- differential-drive model;
- path alignment;
- path-angle handling;
- goal-angle handling;
- footprint / cost-aware trajectory scoring.

Useful especially for dynamic local obstacle handling and smooth local optimization.

## Graceful Controller

Potentially relevant for differential-drive pose convergence and smooth final orientation behavior.

## DWB

Another established local trajectory controller and useful comparison baseline.

None should be introduced before the planner experiment is isolated.

---

# 37. Route Server Role

A warehouse should not rely only on unrestricted free-space navigation.

The environment contains traffic semantics:

```text
one-way aisles
intersections
parking lanes
station ingress
station egress
holding areas
slow zones
restricted zones
```

Long-term structure:

```text
mission destination
    ->
warehouse route graph
    ->
local geometric planner
    ->
controller
```

Example:

```text
Aisle 1
    ->
Intersection C
    ->
Parking lane
    ->
PARK_03 approach
```

The route graph expresses warehouse traffic policy.

The local planner produces the collision-free, physically feasible geometry.

---

# 38. Route Server Is Not Fleet Management

Route Server does not itself solve:

```text
two robots entering the same narrow aisle
two robots requesting one station
parking conflicts
intersection deadlocks
right-of-way
resource reservation
```

Those belong to a fleet-level layer.

Future options:

- custom reservation / fleet manager;
- Open-RMF integration;
- possible VDA 5050 interoperability.

For an initial homogeneous two-robot fleet, a small custom reservation layer may be more practical before adopting a complete fleet framework.

---

# 39. Future Fleet Resource Model

Fleet logic should eventually reserve:

```text
parking slots
stations
narrow aisles
intersections
loading points
chargers
```

Example:

```text
Robot 2 requests PARK_03
        |
        v
Fleet Manager checks resource
        |
        +-- free -> reserve
        |
        +-- occupied -> queue / reroute
```

Independent Nav2 instances should not be expected to resolve fleet resource conflicts implicitly.

---

# 40. Recommended Mature Navigation Architecture

```text
                    Fleet / Mission Manager
                             |
               +-------------+-------------+
               |                           |
        resource reservation         traffic policy
               |                           |
               +-------------+-------------+
                             |
                        Route Server
                             |
                     route / approach
                             |
                     State Lattice
                 feasible SE(2) planning
                             |
                         Controller
                     RPP as baseline
                 other controllers tested
                             |
                  terminal pose validation
                      x / y / yaw
                             |
                optional precision operation
               +-------------+-------------+
               |                           |
              none                  AprilTag docking
                                             |
                                   precision controller
                             |
                        motion interface
                             |
                       PLC / drives
                             |
                 independent safety system
```

---

# 41. Functional Safety Direction

For real deployment, safety must remain independent from ROS navigation.

Future physical architecture should include:

```text
safety lidar
E-stop system
safety PLC / safety controller
STO or safe drive functions
```

Nav2 costmaps, Collision Monitor and controller collision checks are autonomy protections, not the sole safety system.

---

# 42. Simulation Camera Lessons

Earlier high-resolution camera settings caused heavy simulation load.

The current practical camera configuration is approximately:

```text
640 x 480
10 Hz
visualize = false
lazy bridge where possible
```

Lesson:

> sensor configuration should be sized for the task rather than maximized automatically.

Stable timing is more useful for AprilTag docking than unnecessarily high image resolution.

---

# 43. Simulation Timing Lessons

The Gazebo physics step has deliberately been kept small, around:

```text
0.0005 s
```

During navigation / docking debugging this should not be casually changed because timing affects:

- dynamics;
- controller cycles;
- odometry;
- camera timing;
- TF;
- perception.

A real-time factor below 1 also reduces wall-clock sensor frequency even when simulated sensor frequency is configured correctly.

---

# 44. Command Timestamp Warning Observed

Logs showed warnings similar to:

```text
diff_drive_controller:
Ignoring a received command because it is older than cmd_vel_timeout
```

These occurred around some navigation state transitions.

They have not been identified as the cause of the heading oscillation, but command-chain timestamp behavior should be cleaned up before hardware deployment.

---

# 45. Engineering Lessons So Far

## 45.1 Separate symptoms from architecture

Examples:

```text
Docking controller not moving
```

was a command interface mismatch, not bad gains.

```text
Robot oscillating near tagged station
```

is not necessarily perception-related.

## 45.2 Change one subsystem at a time

Useful sequence:

```text
baseline
    ->
one change
    ->
regression test
    ->
interpret result
```

Examples:

- direct DockRobot before mission integration;
- keep RPP during the State Lattice experiment;
- separate perception launch during early integration.

## 45.3 Do not hide structural problems with loose tolerances

Quick workarounds such as:

```text
ignore final yaw
huge yaw tolerance
disable collision checking
arbitrary parking orientation
```

are not acceptable for an industrial system when pose occupancy matters.

## 45.4 Parking orientation can be a real requirement

Uniform heading may affect:

- packing density;
- aisle clearance;
- footprint overlap;
- sensor visibility;
- predictable departure;
- charging / loading geometry.

## 45.5 Precision perception and global navigation are separate layers

AprilTag docking is a final local precision operation, not a replacement for global pose navigation.

## 45.6 Stations need semantics

A destination should eventually represent:

```text
resource identity
pose
tolerances
approach direction
route entry
task
precision method
occupancy behavior
```

---

# 46. Current Repository Checkpoint

Latest confirmed `main`:

```text
fe42d1aa2037103213093c444b8b1fa74708eaf3
```

Commit:

```text
Integrate DockRobot missions and test staging navigation behavior
```

This checkpoint preserves a working system with:

- ordinary Nav2 navigation;
- `RobotExecutor`;
- `MissionManager`;
- DockRobot integration;
- generated docking database;
- AprilTag perception;
- successful precision docking;
- the tested docking-BT experiment;
- the unresolved terminal-heading issue.

This is the reference point for the next planner-architecture work.

---

# 47. Proven Working at This Checkpoint

```text
[PASS] Gazebo robot simulation
[PASS] differential-drive motion
[PASS] wheel odometry pipeline
[PASS] EKF odom transform
[PASS] AMCL saved-map localization
[PASS] ordinary NavigateToPose
[PASS] station configuration loading
[PASS] RobotExecutor GoToStation
[PASS] MissionManager structure
[PASS] runtime dock database generation
[PASS] AprilTag detection
[PASS] tag -> final dock pose transformation
[PASS] direct DockRobot
[PASS] RobotExecutor -> DockRobot integration
[PASS] Station A precision docking
[PASS] Station B ordinary navigation
```

---

# 48. Still Open / Not Yet Proven

```text
[OPEN] deterministic terminal-yaw behavior
[OPEN] State Lattice integration
[OPEN] lattice primitive tuning for this robot
[OPEN] robust repeated pose-navigation test suite
[OPEN] final controller selection
[OPEN] warehouse route graph
[OPEN] one-way aisle / approach policy
[OPEN] multi-robot traffic reservation
[OPEN] parking resource allocation
[OPEN] real PLC command interface
[OPEN] real drive interface
[OPEN] functional safety hardware integration
[OPEN] hardware localization validation
[OPEN] real AprilTag lighting / camera validation
```

---

# 49. Immediate Next Project Plan

## Phase 1 — Return to a clean navigation baseline

Remove / revert the special docking-only BT experiment if we decide not to keep it.

Keep the proven mission and docking stack unchanged.

Goal:

```text
minimize variables before planner comparison
```

## Phase 2 — Add Smac State Lattice

Add a second planner first rather than deleting Smac2D immediately:

```text
GridBased   -> SmacPlanner2D
PosePlanner -> SmacPlannerLattice
```

This enables controlled A/B tests.

The lattice should be appropriate for:

```text
differential-drive kinematics
0.05 m map resolution
actual footprint
realistic turning behavior
```

Do not blindly copy arbitrary motion primitives.

## Phase 3 — Keep RPP unchanged

First test:

```text
State Lattice + current RPP
```

Test:

```text
Home
Station A
Station B
temporary poses
90° goals
near-180° goals
goals approached from both sides
```

## Phase 4 — Instrument navigation

Record:

```text
planned path
path orientations
robot pose
cmd_vel
RPP rotating state
goal error
navigation result
recoveries
completion time
```

This should let us distinguish:

```text
planner issue
controller issue
localization issue
goal-checker issue
```

## Phase 5 — Decide controller

If State Lattice + RPP removes the pathology:

```text
keep RPP
```

unless later requirements justify another controller.

If not, benchmark:

```text
RPP
MPPI
Graceful Controller
possibly DWB
```

with identical tests.

Metrics:

```text
terminal position error
terminal yaw error
oscillation count
path tracking error
completion time
minimum obstacle clearance
command smoothness
CPU load
recovery frequency
```

## Phase 6 — Build warehouse route graph

Once local pose navigation is reliable:

- define aisles;
- define one-way lanes;
- define intersections;
- define station ingress / egress;
- define parking approaches;
- connect to Route Server.

## Phase 7 — Generalize resource schema

Evolve station config from:

```yaml
x:
y:
yaw:
```

toward:

```yaml
pose:
resource:
tolerance:
approach:
task:
precision:
```

Do this after the navigation behavior is understood well enough to know what the schema needs.

## Phase 8 — Multi-Robot Resource Coordination

Add:

- parking reservation;
- exclusive station reservation;
- narrow-aisle coordination;
- intersection coordination;
- deadlock avoidance;
- reliable resource release.

## Phase 9 — Hardware Split

Transition toward:

```text
IPC / RPi5:
    ROS 2 autonomy
    missions
    localization
    routing
    planning
    perception

PLC:
    deterministic process control
    watchdog
    drive handshake
    industrial IO

Safety:
    independent certified layer
```

---

# 50. Recommended Navigation Regression Matrix

| Test | Start | Goal | Heading stress | Expected |
|---|---|---|---:|---|
| T1 | Home | Station B | small | smooth |
| T2 | B | Home | large / possible ~180° | no oscillation |
| T3 | B | A staging | large | oriented feasible approach |
| T4 | A | B | moderate | smooth |
| T5 | random pose | temporary pose | 90° | deterministic |
| T6 | random pose | parking slot | large | deterministic |
| T7 | same XY | opposite yaw | 180° | controlled alignment |
| T8 | narrow approach | parking slot | constrained | no footprint violation |

Every test should be repeated several times.

An intermittent failure is still a failure.

---

# 51. Production Reliability Standard

The system should not be considered robust because:

```text
"it eventually got there"
```

For a warehouse robot, the target is:

```text
predictable
repeatable
bounded
observable
recoverable
safe
```

Several unnecessary 180° rotations followed by eventual success is an unacceptable result even if the action returns success.

---

# 52. Main Technical Conclusion

The terminal-heading issue should **not** be solved with:

```text
AprilTag-specific logic
arbitrary parking headings
loose yaw tolerances
controller gain tuning alone
```

The next correct investigation is:

```text
kinematically feasible pose planning
```

using State Lattice while the rest of the system remains stable.

Only afterward should controller replacement be considered.

---

# 53. Target Architecture Summary

```text
Mission request
      |
      v
Semantic resource
pose / tolerance / task / occupancy
      |
      v
Fleet reservation
      |
      v
Warehouse route graph
      |
      v
Smac State Lattice
kinematically feasible SE(2) path
      |
      v
Local controller
RPP baseline; MPPI / Graceful benchmark if needed
      |
      v
Terminal pose validation
      |
      +------------------------------+
      |                              |
no precision stage             precision station
                                      |
                               AprilTag / other sensor
                                      |
                               precision controller
      |
      v
PLC / drive layer
      |
      v
Independent functional safety
```

---

# 54. Next Concrete Engineering Action

The next implementation task should be:

> **Introduce Smac State Lattice as a second planner while keeping the existing Smac2D and RPP configuration available for controlled comparison.**

Before controller changes, create repeatable pose-navigation tests that deliberately stress final orientation.

This gives the project a defensible path from the current working checkpoint toward real warehouse deployment.

---

# 55. Final Status

The project has moved beyond a basic ROS navigation demo.

The system now has:

- meaningful package separation;
- mission-level actions;
- ordinary station navigation;
- precision docking;
- perception-to-control integration;
- a single station source of truth;
- a known and investigated navigation weakness;
- tested debugging history;
- a more mature long-term warehouse architecture.

The key current lesson is that industrial AMR navigation cannot be designed around isolated destination coordinates alone.

The next version should increasingly reason in terms of:

```text
pose
kinematics
routes
resources
traffic
precision tasks
safety
```

rather than treating every goal as the same generic `NavigateToPose` problem.

That is the project direction as of the `fe42d1a` checkpoint.
