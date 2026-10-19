# Warehouse AMR — Navigation Architecture Decision and Implementation Plan

**Project:** `warehouse_amr`  
**Date:** 2026-10-19  
**Document type:** Current architecture decision record + implementation plan  
**Scope:** Navigation planning, local control, terminal-pose handling, docking integration, warehouse routing, and product-level robustness  
**Primary decision:** **Smac State Lattice + Nav2 MPPI** becomes the target navigation baseline

---

# 1. Purpose

This document records the navigation conclusions reached after the recent terminal-heading investigation, the State Lattice experiment, the comparison with NavFn + Rotation Shim, and the product-level requirements for the AMR.

The purpose is not only to record what will be implemented next, but to preserve the engineering reasoning behind the decision so that future changes can be judged against the same requirements.

The main requirement is:

> The AMR must not be optimized only for the current simulated warehouse. The navigation architecture should be suitable for a reusable industrial AMR product that may later operate in warehouses with different aisle widths, sharp corners, constrained parking areas, difficult station approaches, reverse maneuvers, and multi-robot traffic.

Therefore, the current design priority is:

```text
robustness
predictability
kinematic feasibility
clear subsystem responsibilities
product portability
future extensibility
```

rather than:

```text
quickest configuration that works in the current map
```

---

# 2. Current AMR Characteristics

The navigation architecture is being designed around the actual robot type.

Current platform:

```text
drive type:
    differential drive

approximate body:
    length = 1.0 m
    width  = 0.60 m

navigation footprint:
    x = ±0.50 m
    y = ±0.37 m

wheel separation:
    0.69 m

wheel radius:
    0.10 m

nominal maximum linear velocity:
    ~1.0 m/s drivetrain limit

current navigation target velocity:
    ~0.8 m/s

maximum angular velocity:
    ~1.2 rad/s drivetrain limit
```

The robot is:

- non-circular;
- able to rotate in place;
- intended to carry approximately 50 kg payload;
- intended for structured indoor warehouse operation;
- expected eventually to operate in a multi-robot system;
- expected to use exact position and heading at meaningful destinations.

This makes orientation a real navigation variable, not only a cosmetic final adjustment.

---

# 3. Product-Level Requirement

The project should not assume that every future warehouse will have:

```text
wide aisles
large intersections
easy parking
open loading stations
simple approach geometry
no reversing
no dead ends
```

A sellable AMR platform may need to operate in environments containing:

```text
narrow aisles
tight 90° turns
rack-end turns
constrained loading zones
parking bays
charging stations
dead ends
tight waiting areas
one-way traffic
reverse docking
restricted approach directions
shared intersections
```

The global planner should therefore understand more than free-space XY connectivity.

It should reason about whether the **robot itself** can move through the requested geometry.

---

# 4. Original Navigation Baseline

The original main navigation stack was:

```text
SmacPlanner2D
      |
      v
Regulated Pure Pursuit (RPP)
      |
      v
SimpleGoalChecker
```

This stack worked well for many ordinary navigation cases.

However, a terminal-heading problem appeared during missions such as:

```text
Station B -> Station A staging
```

and later:

```text
Station A -> Home
```

Observed behavior included:

```text
correct XY approach
small final heading corrections
rotation-direction reversal
large ~180° turns
rotation back toward target
eventual success in some runs
```

The same mission could behave differently across runs.

This was unacceptable for an industrial navigation system.

---

# 5. Important Root-Cause Findings Before State Lattice

Several possible causes were investigated and excluded.

## 5.1 AprilTag control was not the cause

The same problem occurred on destinations that did not use AprilTag docking.

Additionally, during B -> A, the large rotations occurred while the robot was still in:

```text
NAVIGATING_TO_STAGING
```

before DockRobot transferred control to the precision docking stage.

Therefore:

> The AprilTag detector and precision docking controller are not the root cause of the terminal-heading problem.

## 5.2 Recovery Spin was not the cause

Captured logs showed normal controller execution and eventual staging success without matching Spin-recovery messages.

Therefore:

> The large rotations were not primarily Nav2's recovery Spin behavior.

## 5.3 Continuous replanning was not the root cause

A special Behavior Tree was tested that replanned only when the path became invalid.

The exact approach trajectory changed, but the terminal oscillation remained.

Therefore:

> Periodic replanning may influence the detailed trajectory, but it is not the root cause.

That experiment was removed.

The Docking Server has returned to the default navigation behavior.

---

# 6. Structural Weakness Identified in SmacPlanner2D + RPP

SmacPlanner2D primarily solves the XY path.

It may produce a path that reaches the correct position while the arrival direction is inconsistent with the requested final yaw.

Example:

```text
robot travels west
        |
        v

goal XY reached

robot heading:
    -X

requested final heading:
    +X
```

The controller then receives a large terminal orientation correction.

This creates an undesirable architectural dependency:

```text
global planner solves position
        |
        v
local controller repairs large orientation mismatch
```

For meaningful warehouse destinations such as parking bays, loading stations, chargers, and waiting poses, final yaw may be a hard constraint.

Therefore, a stronger pose-aware global planning model was required.

---

# 7. State Lattice Was Selected for Evaluation

The chosen global planner was:

```text
nav2_smac_planner::SmacPlannerLattice
```

The reason was not merely docking.

The AMR needs pose-constrained navigation generally.

State Lattice searches over:

```text
x
y
yaw
```

using predefined motion primitives.

For this project, that means the planner can reason about:

```text
forward motion
curved differential-drive motion
orientation changes
rotate-in-place behavior
eventual reverse maneuvers if enabled
```

while checking the actual footprint against the costmap.

---

# 8. State Lattice Is Appropriate for Differential Drive

State Lattice should not be treated as an Ackermann-only planning method.

For Nav2, State Lattice supports configurable motion primitives and can be generated for:

```text
differential drive
Ackermann
omnidirectional
other custom motion sets
```

The project specifically uses a differential-drive lattice.

Current baseline primitive file:

```text
warehouse_diff_5cm_r0.5.json
```

based on the Nav2 Jazzy sample:

```text
5 cm map resolution
differential-drive motion model
16 heading bins
0.5 m forward primitive turning radius
rotate-in-place primitives available
```

The `0.5 m` value is a forward-motion primitive design value.

It is **not** the physical minimum turning radius of the differential-drive robot.

Because the robot can rotate in place:

```text
v = 0
w != 0
```

the planner is still able to represent zero-radius heading changes.

---

# 9. Current State Lattice Planner Role

The planner was promoted from an experimental secondary planner to the main planner.

Current intended global planning chain:

```text
GridBased
    |
    v
SmacPlannerLattice
```

The plugin ID remains:

```text
GridBased
```

for compatibility with the existing Nav2 Behavior Tree.

The name is only the plugin identifier.

The actual plugin is now:

```text
nav2_smac_planner::SmacPlannerLattice
```

---

# 10. Current State Lattice Configuration Direction

Current important concepts:

```text
resolution:
    0.05 m

motion model:
    differential drive

reverse expansion:
    currently disabled

rotate-in-place:
    enabled through lattice primitives

smooth path:
    enabled

actual rectangular footprint:
    used by the costmap / planner collision checker

goal tolerance:
    strict enough to preserve meaningful pose goals
```

Relevant planner parameters currently include approximately:

```yaml
tolerance: 0.10

reverse_penalty: 2.0
change_penalty: 0.05
non_straight_penalty: 1.05
cost_penalty: 2.0
retrospective_penalty: 0.015

rotation_penalty: 5.0

analytic_expansion_ratio: 3.5
analytic_expansion_max_length: 3.0
analytic_expansion_max_cost: 200.0

allow_reverse_expansion: false
smooth_path: true
```

These values are still baseline values.

The primitive set has not yet been deeply tuned for the final production robot.

---

# 11. First State Lattice Result

The first meaningful State Lattice test used:

```text
Smac State Lattice
        +
existing RPP
```

Result:

```text
Home -> Station B
```

completed successfully.

This established that:

```text
[PASS] State Lattice plugin integration
[PASS] lattice file loading
[PASS] basic path generation
[PASS] ordinary path execution
[PASS] compatibility with the existing navigation stack in non-problematic cases
```

The planner itself was therefore not globally broken.

---

# 12. State Lattice + RPP Failure

The next mission:

```text
Station B -> Station A
```

failed during staging navigation.

Observed behavior:

```text
robot reaches target region
        |
        v
begins large rotational / circular behavior
        |
        v
continues turning repeatedly
        |
        v
does not complete staging
        |
        v
precision docking never begins
```

This was more severe than the previous Smac2D terminal-heading issue.

The behavior was effectively:

```text
continuous circling
```

rather than:

```text
eventual success after unnecessary rotations
```

---

# 13. Interpretation of the State Lattice + RPP Failure

This result does **not** justify abandoning State Lattice.

The more defensible conclusion is:

```text
State Lattice produces richer SE(2) paths
        +
the current RPP configuration does not robustly execute
one important orientation-rich path
```

State Lattice may legitimately generate:

```text
forward segment
orientation transition
rotate-in-place segment
forward segment
```

RPP is fundamentally a geometric pure-pursuit controller using a lookahead target.

That controller structure is not always ideal for paths containing tight orientation transitions or repeated poses at approximately the same XY location.

The current RPP configuration also uses relatively large lookahead distances:

```text
lookahead_dist      = 0.8 m
min_lookahead_dist  = 0.5 m
max_lookahead_dist  = 1.2 m
```

Those values were originally chosen for smoother Smac2D paths.

They may cause RPP to look beyond local orientation structure in a lattice path.

---

# 14. Why We Are Not Solving This Only by Tuning RPP

It would be possible to experiment with much smaller RPP lookahead distances.

That might improve the immediate B -> A behavior.

However, the product should eventually perform both:

```text
stable aisle tracking
```

and:

```text
tight orientation-rich maneuvers
```

A solution that depends on finding one narrow Pure Pursuit tuning compromise between those two operating regimes is not preferred.

RPP remains a useful controller and comparison point.

However, it is no longer the preferred long-term controller for the State Lattice architecture.

---

# 15. NavFn + Rotation Shim + MPPI Was Considered

An alternative architecture was discussed:

```text
NavFn
   |
   v
Rotation Shim
   |
   v
MPPI
```

This architecture has several real strengths.

NavFn is:

```text
mature
simple
fast
easy to debug
well suited to open XY path planning
```

Rotation Shim can explicitly handle:

```text
initial heading alignment
final goal heading alignment
```

MPPI can then handle the local differential-drive trajectory.

For many warehouse environments this would work very well.

---

# 16. Why NavFn + Rotation Shim Is Not the Primary Product Architecture

The key limitation is not final yaw.

Rotation Shim can solve start and final orientation well.

The deeper limitation is:

> NavFn does not guarantee that the entire global path is kinematically feasible for the actual robot.

NavFn primarily reasons about XY connectivity.

A path such as:

```text
──────────┐
          │
          │
```

may be perfectly valid geometrically while still requiring a difficult or impossible orientation transition for a long rectangular robot in a tight corner.

With NavFn, this question is deferred:

```text
Can the robot physically execute this path?
```

from the global planner to the local controller.

That creates a failure mode such as:

```text
NavFn:
    path found

robot begins motion

MPPI:
    cannot find acceptable local trajectory

robot:
    slows / stops / oscillates / triggers recovery
```

For a commercial AMR, this is less desirable than rejecting the route before motion starts.

---

# 17. Planning-Time Feasibility Is a Major Product Advantage

State Lattice can reject a path if no sequence of allowed motion primitives and footprint orientations can connect start and goal through the current costmap.

Desired behavior:

```text
planner:
    no feasible path exists

robot:
    remains stationary

mission / fleet layer:
    wait
    reroute
    select another approach
    report resource blocked
```

This is preferable to:

```text
planner:
    path found

robot:
    enters constrained area

controller:
    path cannot be executed reliably

robot:
    stops inside traffic area
```

For a fleet system, this difference becomes even more important.

Planning-time failure is:

```text
bounded
observable
actionable
```

Controller-time geometric failure can be:

```text
late
traffic-blocking
harder to diagnose
more difficult to recover from
```

This is one of the strongest reasons State Lattice remains the primary planner.

---

# 18. Why Current Warehouse Geometry Must Not Drive the Architecture

The current simulation may contain relatively generous free space.

That is irrelevant to the product-level planner decision.

The same AMR may later be deployed into:

```text
Warehouse A:
    wide aisles

Warehouse B:
    narrow rack corridors

Warehouse C:
    constrained parking

Warehouse D:
    reverse loading bays

Warehouse E:
    shared intersections with multiple robots
```

The software architecture should not need to be redesigned for each warehouse simply because the global planner assumed easy geometry.

Warehouse-specific configuration should change:

```text
map
route graph
semantic resources
speed zones
traffic rules
station definitions
```

The fundamental planning model should remain strong enough for all of them.

---

# 19. State Lattice and Sharp 90° Turns

A sharp map-level 90° route does not mean the robot should simply receive a geometric right-angle path.

State Lattice can search for an executable maneuver such as:

```text
        ↑
       /
      /
─────╯
```

or:

```text
──────●
      ↻
      │
      ↑
```

depending on:

```text
available space
footprint clearance
motion primitives
rotation penalty
obstacles
```

The planner therefore has explicit responsibility for determining whether the turn is feasible.

This is an important product-level capability.

---

# 20. State Lattice and Future Reverse Maneuvers

Reverse expansion is currently disabled:

```yaml
allow_reverse_expansion: false
```

This is intentional for the present stage.

However, future warehouse requirements may include:

```text
reverse parking
reverse station approach
backing out of a dead end
three-point-like maneuvers
constrained charger approach
```

State Lattice can later represent those maneuvers explicitly.

A future path may contain:

```text
forward
   |
   v
turn / align
   |
   v
direction inversion
   |
   v
reverse into resource
```

That maneuver can be planned globally rather than discovered accidentally by the local controller.

MPPI also contains mechanisms for respecting path inversion points from feasible planners.

Therefore State Lattice + MPPI provides a path toward more complex future maneuvering without changing the overall architecture.

---

# 21. Rotation Shim Conclusion

Rotation Shim remains a valid Nav2 tool.

It is especially useful when the global planner does not provide a kinematically feasible orientation sequence, such as:

```text
NavFn
Smac2D
Theta*
```

In those systems the shim can compensate for a large initial heading mismatch and optionally enforce final goal yaw.

However, it should not be foundational to our primary architecture because:

```text
State Lattice already represents heading throughout the path
```

and:

```text
MPPI can consume that information directly
```

Rotation Shim may still be useful later for:

```text
special behaviors
comparison testing
fallback modes
specific operational policies
```

but it is not required merely to repair the global planner's lack of orientation awareness.

---

# 22. Final Global Planner Decision

The product-level primary global planner is:

```text
Smac State Lattice
```

Reasons:

- supports differential-drive motion primitives;
- reasons in SE(2): x, y, yaw;
- uses robot-aware kinematic motion;
- works with the rectangular footprint;
- supports rotate-in-place behavior;
- can later support reverse maneuvers;
- can reject impossible geometry at planning time;
- produces meaningful orientation throughout the path;
- is suitable for constrained warehouse navigation;
- does not assume wide aisles or easy corner geometry.

NavFn remains:

```text
secondary / benchmark / fallback candidate
```

rather than the primary architecture.

---

# 23. Final Local Controller Direction

The next local controller is:

```text
nav2_mppi_controller::MPPIController
```

with:

```text
motion_model = DiffDrive
```

The intended combination is:

```text
Smac State Lattice
        |
        v
Nav2 MPPI
```

This is the next major architecture implementation.

---

# 24. Why MPPI Fits State Lattice Better

State Lattice intentionally generates an orientation-aware feasible path.

MPPI evaluates candidate local trajectories using multiple objectives.

Relevant MPPI responsibilities include:

```text
kinematic constraints
path following
path alignment
path orientation
goal position
goal orientation
obstacle cost
trajectory feasibility
velocity limits
acceleration limits
```

This creates a more natural relationship:

```text
State Lattice:
    defines globally feasible pose sequence

MPPI:
    optimizes local executable trajectory around that sequence
```

rather than:

```text
State Lattice:
    defines SE(2) path

RPP:
    reduces execution largely to a geometric lookahead carrot
```

---

# 25. MPPI Path Orientation Handling

The planned configuration will deliberately use the orientation information generated by State Lattice.

Important concepts:

```yaml
PathAlignCritic:
    use_path_orientations: true
```

and:

```yaml
PathAngleCritic:
    mode: 2
```

For Nav2 Jazzy:

```text
mode 2 =
CONSIDER_FEASIBLE_PATH_ORIENTATIONS
```

This means the local controller does not need to infer intended motion direction only from XY geometry.

It can use the path yaw generated by the feasible planner.

This is one of the strongest architectural reasons to pair MPPI with State Lattice.

---

# 26. Initial MPPI Motion Policy

The first MPPI baseline should match the current global planner policy.

State Lattice currently does not allow reverse expansion.

Therefore MPPI should initially use:

```text
vx_min = 0
```

so it does not invent reverse trajectories that the global planner never requested.

Initial concept:

```text
motion_model = DiffDrive

vx_max ~= 0.8 m/s
vx_min = 0.0 m/s
wz_max ~= 1.0 rad/s

ax_max ~= 0.8 m/s²
ax_min ~= -0.8 m/s²
az_max ~= 1.0 rad/s²
```

These are starting values.

They are not yet final hardware-certified limits.

---

# 27. Initial MPPI Horizon

Current controller server frequency:

```text
20 Hz
```

Therefore:

```text
controller period = 0.05 s
```

MPPI should initially use:

```yaml
model_dt: 0.05
```

A starting horizon of:

```yaml
time_steps: 56
```

gives approximately:

```text
56 * 0.05 = 2.8 seconds
```

of prediction.

This provides enough horizon for meaningful trajectory evaluation without making the controller unnecessarily expensive.

---

# 28. Initial MPPI Critic Structure

Initial critic set:

```text
ConstraintCritic
CostCritic
GoalCritic
GoalAngleCritic
PathAlignCritic
PathFollowCritic
PathAngleCritic
```

Each has a specific responsibility.

## ConstraintCritic

Enforces consistency with the differential-drive control limits.

## CostCritic

Evaluates costmap and footprint collision risk.

The project should use:

```yaml
consider_footprint: true
```

because the AMR is rectangular and large enough that point-robot obstacle scoring is not sufficient.

## GoalCritic

Encourages convergence to requested goal position.

## GoalAngleCritic

Encourages convergence to requested final heading.

For this AMR:

```yaml
symmetric_yaw_tolerance: false
```

because:

```text
yaw
```

and:

```text
yaw + pi
```

are not generally equivalent parking or station orientations.

## PathAlignCritic

Encourages the predicted trajectory to remain aligned with the global path.

Planned:

```yaml
use_path_orientations: true
```

## PathFollowCritic

Encourages progress along the path.

## PathAngleCritic

Handles large heading mismatch relative to the feasible global path.

Planned:

```yaml
mode: 2
```

so that feasible State Lattice path orientations are considered.

---

# 29. Why PreferForwardCritic Is Not Initially Required

MPPI examples commonly include:

```text
PreferForwardCritic
```

However, the first project configuration will already enforce:

```text
vx_min = 0
```

Therefore reverse motion is physically excluded from sampled controls after constraints.

Adding another cost term to discourage reverse motion would be redundant.

Later, if reverse motion is intentionally enabled:

```text
vx_min < 0
allow_reverse_expansion = true
```

then `PreferForwardCritic` may become useful to express:

```text
reverse allowed
but forward preferred
```

rather than:

```text
reverse forbidden
```

---

# 30. State Lattice + MPPI Responsibilities

The intended subsystem boundary is:

```text
State Lattice:
    "Find a globally feasible sequence of robot poses."

MPPI:
    "Find the best locally executable trajectory through that sequence."

Velocity smoother:
    "Condition the outgoing command."

Collision Monitor:
    "Provide operational collision intervention."

PLC / drives:
    "Execute deterministic drive commands."

Independent safety system:
    "Provide certified safety functions."
```

These responsibilities should remain separate.

---

# 31. Dynamic Obstacles Do Not Remove the Need for Global Feasibility

State Lattice feasibility does not mean the future real trajectory is guaranteed.

The global planner operates with:

```text
current map
current costmap
current footprint
configured motion primitives
```

The environment may change after planning.

Examples:

```text
forklift enters aisle
person steps into path
pallet is moved
localization changes
wheel slip occurs
another AMR blocks an intersection
```

Therefore MPPI and local costmaps remain essential.

The architecture is:

```text
global kinematic feasibility
        +
local dynamic adaptation
```

not one instead of the other.

---

# 32. Docking Architecture Remains Separate

The precision docking architecture remains valid and should not be merged into general navigation.

For Station A:

```text
mission requests Station A
        |
        v
DockRobot
        |
        v
State Lattice + MPPI
navigate to staging pose
        |
        v
staging success
        |
        v
initial AprilTag perception
        |
        v
detected_dock_pose refinement
        |
        v
Docking Server precision controller
        |
        v
final dock pose
```

The global navigation stack should deliver the robot reliably to staging.

The precision controller then handles the final short-range station-specific alignment.

---

# 33. Parking and Other Non-Docking Goals

Parking should not be reduced to a position-only goal.

A parking resource may require:

```text
x
y
yaw
```

for:

```text
packing density
aisle clearance
predictable departure
sensor orientation
fleet resource occupancy
charger geometry
loading alignment
```

State Lattice + MPPI should therefore handle exact parking pose navigation directly.

No AprilTag is required unless the application needs higher local precision than normal localization can provide.

---

# 34. Future Semantic Destination Model

The station model should eventually evolve toward semantic resources.

Concept:

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
    route_node: park_03_entry
    allowed_direction: forward

  task:
    type: parking

  precision:
    type: none
```

A docking station could use:

```yaml
precision:
  type: apriltag_docking
```

This allows the same navigation architecture to support:

```text
parking
loading
waiting
charging
inspection
docking
```

without treating every station as a special code path.

---

# 35. Route Server Role

State Lattice should not be expected to encode warehouse traffic policy by itself.

The future warehouse needs a route layer describing:

```text
aisles
one-way lanes
intersections
station ingress
station egress
parking lanes
restricted regions
slow zones
holding areas
```

Target structure:

```text
Mission destination
        |
        v
Route Server / route graph
        |
        v
allowed corridor / route segment
        |
        v
State Lattice
        |
        v
MPPI
```

The route graph decides:

```text
where the robot is legally allowed to travel
```

State Lattice decides:

```text
how the robot can physically travel through that region
```

MPPI decides:

```text
how to execute the local motion now
```

---

# 36. Fleet Layer Role

Route Server is not fleet management.

A multi-robot system will also require:

```text
resource reservation
intersection coordination
narrow-aisle locking
parking reservation
station reservation
charger reservation
deadlock avoidance
right-of-way logic
```

Future hierarchy:

```text
Fleet Manager
      |
      v
resource / traffic decisions
      |
      v
Route Server
      |
      v
State Lattice
      |
      v
MPPI
```

This separation is important for eventual productization.

---

# 37. Safety Architecture

Navigation robustness must not be confused with certified functional safety.

The intended real robot still requires an independent safety system.

Concept:

```text
ROS 2 autonomy:
    map
    localization
    planning
    MPPI
    docking
    missions

operational collision layer:
    local costmap
    controller collision scoring
    Collision Monitor

functional safety:
    safety lidar
    E-stop
    safety PLC / safety controller
    STO / safe drive functions
```

A good Nav2 stack does not replace industrial safety functions.

---

# 38. Why We Are Not Optimizing for the Current Simulation

The current simulation is used to validate architecture.

It is not the product specification.

Therefore decisions should not be based on statements such as:

```text
"the current aisle is wide enough"
"the current station has enough room"
"this map has no reverse parking"
```

Instead, decisions should answer:

```text
Would this still make sense
if the next customer warehouse is harder?
```

For State Lattice + MPPI, the answer is currently stronger than for NavFn + Rotation Shim as the default platform architecture.

---

# 39. NavFn's Future Role

NavFn is not being rejected as a poor planner.

It remains valuable for:

```text
comparison testing
debugging
simple robot variants
resource-constrained hardware
fallback configurations
very open facilities
circular robots
```

If later product data proves that State Lattice adds unnecessary compute or complexity in a particular hardware tier, NavFn could be offered as a simplified navigation profile.

However, the main AMR architecture should not downgrade its global feasibility model prematurely.

---

# 40. Rotation Shim's Future Role

Rotation Shim may still be useful for:

```text
specific behaviors
fallback controller chains
simplified NavFn configuration
special terminal alignment policies
benchmarking
```

But the primary navigation stack should not require the shim to compensate for a planner that ignores heading throughout the global path.

The preferred primary stack is:

```text
State Lattice
        +
MPPI
```

without Rotation Shim as a required architectural layer.

---

# 41. Current Rejected Shortcuts

The following should not be used as structural fixes:

```text
increase yaw tolerance dramatically
ignore final orientation
convert parking to position-only goals
disable collision checking
flip station orientation to hide approach problems
make Station A use a one-off planner hack
remove rotate-in-place primitives simply because RPP struggled
tune RPP blindly until one test passes
choose NavFn only because the current warehouse is open
```

These approaches may hide symptoms without producing a robust product architecture.

---

# 42. Current Open Technical Questions

The following still require engineering validation:

```text
[OPEN] exact MPPI critic tuning
[OPEN] State Lattice primitive tuning for final hardware
[OPEN] ideal rotation penalty
[OPEN] reverse-motion product policy
[OPEN] reverse primitive configuration
[OPEN] path inversion handling
[OPEN] narrow-aisle performance
[OPEN] constrained 90° corner performance
[OPEN] parking-bay performance
[OPEN] controller CPU load on target IPC / RPi5
[OPEN] dynamic obstacle behavior
[OPEN] final route graph design
[OPEN] fleet reservation protocol
[OPEN] hardware localization accuracy
[OPEN] real wheel-slip influence
```

These are normal future engineering tasks.

They do not invalidate the current architectural direction.

---

# 43. Validation Philosophy Going Forward

The project should not move through unnecessary micro-tests after every parameter change.

The preferred process is:

```text
implement one meaningful subsystem-level change
        |
        v
run representative mission scenarios
        |
        v
evaluate behavior
        |
        v
instrument deeply only if results are ambiguous or wrong
```

The goal is not to prove every configuration line individually.

The goal is to validate meaningful engineering milestones.

---

# 44. Immediate Implementation Step

The next implementation is:

> Replace RPP with Nav2 MPPI while keeping Smac State Lattice and all other major subsystems unchanged.

The following remain unchanged initially:

```text
State Lattice planner
lattice primitive file
AMCL
EKF
global costmap
local costmap
SimpleGoalChecker
velocity smoother
Collision Monitor
RobotExecutor
MissionManager
DockRobot
AprilTag detector
dock_pose_publisher
Docking Server precision controller
```

The major change is:

```text
RPP
 |
 v
MPPI
```

---

# 45. Initial MPPI Configuration Direction

Initial conceptual configuration:

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"

  motion_model: "DiffDrive"

  model_dt: 0.05
  time_steps: 56
  batch_size: 2000
  iteration_count: 1

  vx_max: 0.8
  vx_min: 0.0
  vy_max: 0.0

  wz_max: 1.0

  ax_max: 0.8
  ax_min: -0.8
  az_max: 1.0

  PathAlignCritic:
    use_path_orientations: true

  PathAngleCritic:
    mode: 2

  GoalAngleCritic:
    symmetric_yaw_tolerance: false

  CostCritic:
    consider_footprint: true
```

These values are the first baseline, not final tuning.

---

# 46. First MPPI Mission Regression

After MPPI is integrated, run:

```text
Home -> Station B
Station B -> Station A
Station A -> Station B
Station B -> Home
```

This is intentionally compact.

The most important cases are:

```text
Home -> B
```

to verify normal aisle behavior, and:

```text
B -> A
```

to test the path that caused State Lattice + RPP to circle indefinitely.

---

# 47. Initial Success Criteria

The first MPPI result should be judged primarily by architecture-level behavior.

Required:

```text
no indefinite circling
no repeated unnecessary ~180° reversals
no continuous left/right heading oscillation
normal progress along ordinary routes
correct final pose orientation
successful staging completion
clean transition into DockRobot precision phase
```

Minor issues such as:

```text
slightly conservative speed
slightly wide corner
small steering roughness
```

can be tuned later.

The initial goal is to establish a stable planner-controller combination.

---

# 48. If State Lattice + MPPI Works

If the mission sequence is clean:

```text
State Lattice + MPPI
```

becomes the new navigation baseline.

Then proceed to:

```text
MPPI performance tuning
lattice tuning
narrow-space scenarios
parking scenarios
route graph
semantic resources
fleet coordination
hardware transition
```

The planner/controller architecture should then remain stable while the system grows around it.

---

# 49. If State Lattice + MPPI Still Fails

If B -> A or another orientation-rich case still fails, the next task is **not** to abandon State Lattice immediately.

Inspect the actual global path.

Record:

```text
path XY
path yaw
rotate-in-place segments
primitive transitions
path replans
goal orientation
MPPI transformed global plan
cmd_vel
robot pose
```

Determine whether:

```text
A) State Lattice generated a pathological path

or

B) MPPI failed to execute a reasonable path
```

Only then change the responsible subsystem.

---

# 50. If the Lattice Path Is the Problem

Possible future lattice work:

```text
increase heading resolution
change forward turning radius
change rotation penalty
regenerate custom differential-drive primitives
enable / tune reverse motion
change analytic expansion parameters
change cost penalties
constrain approach corridors through routing
```

The primitive file shipped with Nav2 is a baseline.

A commercial AMR may eventually require a lattice generated specifically around:

```text
real wheelbase
real footprint
preferred maneuver shapes
real drive limits
operational policies
```

---

# 51. If MPPI Is the Problem

If the path is reasonable but MPPI behaves poorly, tune:

```text
prediction horizon
batch size
sampling standard deviations
path alignment weight
path angle weight
goal angle weight
cost critic
velocity limits
acceleration limits
prune distance
near-goal thresholds
```

The controller should be tuned against the same planner path rather than changing both planner and controller simultaneously.

---

# 52. Later Robustness Scenarios

After the basic mission regression works, use a compact but meaningful robustness matrix.

Recommended scenarios:

```text
1. normal long aisle
2. large final yaw change
3. same XY / opposite yaw
4. constrained 90° corner
5. narrow aisle turn
6. parking bay
7. dead-end turnaround
8. obstacle partially blocking route
9. dynamic obstacle crossing
10. docking staging approach
```

These are product-relevant scenarios.

They are more useful than dozens of parameter-isolation micro-tests.

---

# 53. Future Reverse-Motion Milestone

Reverse motion should be introduced intentionally, not accidentally.

Future coordinated change:

```text
State Lattice:
    allow_reverse_expansion = true

MPPI:
    vx_min < 0

Path inversion handling:
    enabled / tuned

mission / safety policy:
    define where reverse is allowed
```

Potential additional critic:

```text
PreferForwardCritic
```

so that:

```text
reverse is permitted
but forward travel remains preferred
```

This should be treated as a separate milestone.

---

# 54. Product Architecture Target

The intended mature navigation architecture is:

```text
                    Fleet / Mission Manager
                              |
                              v
                    Semantic Resource Layer
               pose / tolerance / task / occupancy
                              |
                              v
                       Route / Traffic Layer
                  legal aisles / direction / zones
                              |
                              v
                     Smac State Lattice
                globally feasible SE(2) planning
                              |
                              v
                           MPPI
              local differential-drive trajectory
                              |
                              v
                     Velocity Conditioning
                              |
                              v
                     Collision Monitor
                              |
                              v
                      PLC / Drive Layer
                              |
                              v
                         Motor Drives


Independent safety path:

Safety lidar / E-stop / safety PLC
              |
              v
        STO / safe motion
```

Precision destination branch:

```text
State Lattice + MPPI
        |
        v
staging pose
        |
        v
precision task required?
        |
        +-----------------------+
        |                       |
       no                      yes
        |                       |
        v                       v
goal complete             AprilTag / sensor
                                |
                                v
                        Docking controller
                                |
                                v
                           final pose
```

---

# 55. Core Design Principles Going Forward

The following principles should guide future architecture decisions.

## 55.1 Model the actual robot

Do not reduce a rectangular differential-drive AMR to a point if the real geometry matters.

## 55.2 Fail early when possible

A planner rejecting an impossible maneuver is preferable to a controller discovering it after entering the maneuver.

## 55.3 Preserve meaningful yaw

Parking, loading, charging, and station orientations are real constraints.

## 55.4 Keep global and local responsibilities separate

Global planner:

```text
global kinematic feasibility
```

Local controller:

```text
real-time trajectory execution
```

## 55.5 Do not let one warehouse define the product

The navigation stack should tolerate more difficult customer layouts without architectural replacement.

## 55.6 Do not confuse operational collision avoidance with functional safety

Nav2 protects autonomy behavior.

Certified safety remains independent.

## 55.7 Prefer architecture fixes over parameter hacks

Do not hide structural weaknesses with loose tolerances or station-specific exceptions.

---

# 56. Current Decision Summary

The navigation decisions at this point are:

```text
GLOBAL PLANNER
    Smac State Lattice
    KEEP

CURRENT CONTROLLER
    RPP
    REPLACE

NEXT CONTROLLER
    MPPI DiffDrive

ROTATION SHIM
    not part of primary architecture
    retain as optional tool

NAVFN
    not primary product planner
    retain as benchmark / fallback option

APRILTAG DOCKING
    keep as final precision layer

ROUTE SERVER
    future warehouse traffic layer

FLEET MANAGER
    future reservation / coordination layer

FUNCTIONAL SAFETY
    independent from ROS navigation
```

---

# 57. Main Conclusion

The central conclusion is:

> **The product should prefer a globally robot-aware, kinematically feasible planner and a local predictive controller, rather than relying on a simpler XY planner and pushing difficult vehicle geometry into the controller.**

For this AMR, that means:

```text
Smac State Lattice
        +
MPPI DiffDrive
```

as the primary navigation architecture.

The fact that State Lattice + RPP failed during B -> A does not invalidate State Lattice.

It demonstrated that the richer SE(2) path requires a controller that can consume and optimize around that information more appropriately.

The next implementation therefore tests the intended pairing directly:

```text
State Lattice
        +
MPPI
```

---

# 58. Immediate Next Action

The next concrete engineering task is:

> **Replace the current Regulated Pure Pursuit `FollowPath` controller with Nav2 MPPI configured for differential drive and feasible State Lattice path orientations.**

Then run:

```text
Home -> B -> A -> B -> Home
```

If that sequence is stable, the project will adopt:

```text
State Lattice + MPPI
```

as the new navigation baseline and move on to:

```text
controller tuning
lattice tuning
constrained-space testing
semantic resource modeling
Route Server integration
fleet coordination
hardware transition
```

---

# 59. Current Project Direction in One Diagram

```text
                        SELLABLE AMR PLATFORM
                                |
                                v
                   robust warehouse navigation
                                |
                +---------------+---------------+
                |                               |
                v                               v
       traffic / resource policy       physical path feasibility
                |                               |
                v                               v
          Route / Fleet layer           Smac State Lattice
                                                |
                                                v
                                             MPPI
                                                |
                                                v
                                      collision / command chain
                                                |
                                                v
                                         PLC / motor drives
                                                |
                                                v
                                      independent safety


Special precision task:

State Lattice + MPPI
        |
        v
reach station staging
        |
        v
AprilTag / local sensor
        |
        v
precision docking controller
```

This is the implementation direction as of 2026-10-19.
