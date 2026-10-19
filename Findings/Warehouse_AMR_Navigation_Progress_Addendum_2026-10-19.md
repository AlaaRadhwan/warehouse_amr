# Warehouse AMR — Navigation Architecture Progress Addendum

**Project:** `warehouse_amr`  
**Date:** 2026-10-19  
**Purpose:** Record all findings and decisions made after the 2026-10-18 comprehensive project documentation.  
**Previous project checkpoint:** `fe42d1aa2037103213093c444b8b1fa74708eaf3`  
**Previous checkpoint commit:** `Integrate DockRobot missions and test staging navigation behavior`

---

# 1. Scope of This Addendum

The previous documentation ended with the unresolved terminal-heading problem and the plan to evaluate Smac State Lattice while initially keeping RPP unchanged.

Since then, the project has:

- removed the temporary docking-only Behavior Tree experiment;
- switched the global planner from SmacPlanner2D to SmacPlannerLattice;
- adopted a differential-drive state lattice matching the 5 cm costmap;
- intentionally kept RPP unchanged for the first planner-level test;
- successfully completed ordinary navigation to Station B;
- reproduced a much more severe navigation failure during B -> A staging;
- identified a likely mismatch between State Lattice's richer SE(2) path structure and the current RPP configuration / execution behavior;
- revised the controller plan;
- selected MPPI as the next controller to evaluate with State Lattice.

This addendum focuses only on work and conclusions made after the previous documentation.

---

# 2. Starting Point

The relevant previous navigation stack was:

```text
SmacPlanner2D
      |
      v
RPP
      |
      v
SimpleGoalChecker
```

Known problem:

```text
orientation-sensitive goal
        |
        v
2D planner reaches XY
        |
        v
RPP performs large terminal heading correction
        |
        v
intermittent left/right / ~180° oscillation
```

The issue occurred both:

```text
B -> A staging
```

and:

```text
A -> Home
```

so it was already known to be a general navigation problem rather than an AprilTag-specific problem.

---

# 3. Docking-Specific BT Experiment Was Removed

A previous debugging experiment configured Docking Server staging navigation to use:

```text
navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml
```

instead of the normal Nav2 `NavigateToPose` Behavior Tree.

That experiment had changed the approach geometry but did not remove the terminal oscillation.

Therefore it was removed before the State Lattice work.

The following override was removed:

```text
docking_server.ros__parameters.navigator_bt_xml
```

Runtime verification after cleanup:

```bash
ros2 param get /docking_server navigator_bt_xml
```

returned:

```text
String value is:
```

meaning the Docking Server no longer injects a custom navigation BT.

This restores the clean default behavior:

```text
DockRobot
   |
   v
NavigateToPose
   |
   v
normal bt_navigator default tree
```

This was important because the next planner experiment needed to test the normal navigation stack rather than a special docking-only variant.

---

# 4. State Lattice Was Promoted to the Main Planner

An intermediate idea was to keep:

```text
GridBased   -> SmacPlanner2D
PosePlanner -> SmacPlannerLattice
```

and compare the two manually.

This was reconsidered.

Because the project already had a safe Git checkpoint, and because State Lattice had been selected as the intended long-term pose planner, keeping it inactive would only delay the real system-level test.

The architecture was therefore changed directly to:

```text
GridBased
    |
    v
SmacPlannerLattice
    |
    v
RPP
```

The planner plugin ID was deliberately kept as:

```text
GridBased
```

because the stock Nav2 Behavior Tree already uses that planner ID.

The ID is only a plugin name; it does not require the plugin to be SmacPlanner2D.

This avoided creating another custom BT solely to select a new planner.

---

# 5. Differential-Drive State Lattice

The selected initial lattice was the standard Nav2 Jazzy differential-drive primitive set:

```text
resolution       = 0.05 m
motion model     = differential drive
turning radius   = 0.50 m
headings         = 16
rotate-in-place  = supported
```

Source file:

```text
/opt/ros/jazzy/share/nav2_smac_planner/
sample_primitives/
5cm_resolution/
0.5m_turning_radius/
diff/
output.json
```

A project-owned copy was created:

```text
src/amr_navigation/config/lattice/
warehouse_diff_5cm_r0.5.json
```

The project therefore no longer depends on an implicit planner asset hidden in `/opt/ros`.

The lattice is version-controlled with the AMR navigation configuration.

---

# 6. Why the 0.5 m Lattice Is Only an Initial Baseline

For a differential-drive robot, the `0.5 m` turning radius should not be interpreted as the robot's physical minimum turning radius in the same way as an Ackermann vehicle.

The selected `diff` lattice also contains rotate-in-place primitives.

Therefore the robot can still execute:

```text
zero translation
+
nonzero angular velocity
```

The 0.5 m value mainly affects the curvature of the forward-moving lattice primitives.

This value is currently a reasonable initial motion-shape preference, not a final hardware constraint.

Future tuning may alter:

```text
turning radius
number of headings
primitive geometry
rotation penalty
reverse expansion
```

but only if testing shows a clear need.

---

# 7. State Lattice Planner Configuration

The planner configuration was changed conceptually to:

```yaml
planner_server:
  ros__parameters:

    planner_plugins:
      - "GridBased"

    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerLattice"
      lattice_filepath: <runtime rewritten path>

      tolerance: 0.10
      allow_unknown: true

      max_iterations: 1000000
      max_on_approach_iterations: 1000
      terminal_checking_interval: 5000
      max_planning_time: 2.0

      reverse_penalty: 2.0
      change_penalty: 0.05
      non_straight_penalty: 1.05
      cost_penalty: 2.0
      retrospective_penalty: 0.015

      rotation_penalty: 5.0

      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      analytic_expansion_max_cost: 200.0
      analytic_expansion_max_cost_override: false

      lookup_table_size: 20.0
      cache_obstacle_heuristic: false

      allow_reverse_expansion: false

      smooth_path: true
      debug_visualizations: false
```

Important current decisions:

```text
reverse driving        = disabled
rotate-in-place        = available
RPP                    = unchanged
goal checker           = unchanged
mission architecture   = unchanged
docking controller     = unchanged
```

---

# 8. Runtime Lattice Path Handling

`simulation_navigation.launch.py` resolves the project-owned lattice file using the installed `amr_navigation` package path.

Conceptually:

```python
lattice_file = os.path.join(
    navigation_share,
    "config",
    "lattice",
    "warehouse_diff_5cm_r0.5.json",
)
```

Then Nav2 receives:

```text
planner_server.ros__parameters.GridBased.lattice_filepath
```

through the existing `RewrittenYaml` runtime configuration.

This preserves portability.

No absolute developer-machine path is stored in `nav2_params.yaml`.

---

# 9. Why RPP Was Initially Kept

The first State Lattice experiment intentionally changed only the planner.

The reasoning was:

```text
old:
SmacPlanner2D + RPP

test:
SmacPlannerLattice + same RPP
```

This isolated whether the original problem was mainly caused by the 2D planner's lack of heading-aware path generation.

If both planner and controller had been changed at the same time, it would have been difficult to know which component actually solved or caused a behavior change.

This was a useful system-level isolation step, not an intention to keep RPP indefinitely.

---

# 10. First State Lattice Test Result

With:

```text
SmacPlannerLattice
+
existing RPP
```

ordinary navigation to Station B completed successfully.

This proves several important things:

```text
[PASS] State Lattice plugin loads
[PASS] lattice file is valid
[PASS] planner can generate executable paths
[PASS] RPP can follow at least ordinary lattice paths
[PASS] the new planner did not globally break navigation
```

This is important because the later failure is therefore not simply:

```text
"State Lattice does not work"
```

The failure is scenario-dependent.

---

# 11. B -> A Failure With State Lattice + RPP

After reaching Station B successfully, the robot was commanded to Station A.

Because Station A is docking-enabled, the current mission state became:

```text
NAVIGATING_TO_STAGING
```

During this stage, the robot began turning continuously in circles near the target region.

Observed behavior:

```text
approach staging region
      |
      v
rotation / curved motion begins
      |
      v
robot continues circling
      |
      v
does not complete staging
      |
      v
does not enter precision docking
```

Unlike the earlier Smac2D behavior, which often eventually finished after unnecessary large rotations, this run did not naturally resolve.

The new failure can therefore be classified as:

```text
persistent / effectively unbounded rotational navigation behavior
```

rather than merely:

```text
intermittent terminal-heading oscillation
```

---

# 12. What the Screenshots Show

The captured RViz / Gazebo views show the robot repeatedly changing orientation around the staging area while the mission feedback remains:

```text
state: NAVIGATING_TO_STAGING
```

This is important.

The precision docking phase has not started.

Therefore the motion is still produced by:

```text
NavigateToPose
+
global planner
+
local controller
```

and not by:

```text
AprilTag docking controller
```

This further confirms the earlier conclusion:

> The AprilTag system is not responsible for the navigation oscillation / circling problem.

---

# 13. The Special Docking BT Is Also Excluded

The custom docking navigation BT had already been removed before this test.

Therefore this failure cannot be explained by:

```text
replan-only-when-invalid docking BT
```

or by any special docking-only planner logic.

The system under test was the clean default navigation flow.

This removes another previously plausible variable.

---

# 14. New Failure Interpretation

The working hypothesis has now changed.

The State Lattice planner is capable of producing more expressive differential-drive paths than SmacPlanner2D.

A differential-drive lattice may contain:

```text
forward arcs
straight motion
orientation transitions
rotate-in-place primitives
```

That is desirable.

However, RPP is fundamentally a geometric pure-pursuit controller centered on a lookahead point.

Our current RPP configuration uses:

```text
lookahead_dist      = 0.8 m
min_lookahead_dist  = 0.5 m
max_lookahead_dist  = 1.2 m
lookahead_time      = 1.5 s

use_velocity_scaled_lookahead_dist = true
use_rotate_to_heading              = true
```

These values were reasonable for relatively smooth SmacPlanner2D paths.

They may be much less appropriate for State Lattice paths containing tight heading transitions or rotate-in-place segments.

---

# 15. Relevant RPP / State Lattice Behavior Found in Nav2 History

Nav2 has previously had to add explicit RPP handling for State Lattice rotate-in-place cusps.

An older issue documented that RPP originally failed to detect in-place rotation as a cusp because consecutive poses could share the same XY location.

A later Nav2 change added support for rotate-in-place cusps and adjusted path pruning behavior for repeated poses at the same location.

This is relevant because it proves that:

```text
State Lattice rotate-in-place path structure
```

requires special handling from a pure-pursuit controller.

---

# 16. Remaining RPP Limitation Relevant to Our Failure

A later RPP issue described another important behavior:

```text
paths with sharp / near-in-place rotations
+
Euclidean lookahead selection
```

can cause the controller to effectively look beyond the intended local rotation segment.

The reporter specifically noted that very small lookahead values were required for such paths to work reliably in their case.

This matters directly to our configuration because our minimum lookahead is currently:

```text
0.5 m
```

and the nominal lookahead is:

```text
0.8 m
```

For a path whose meaningful maneuver may occur almost at one XY location, a half-meter or larger carrot can skip over the local rotational structure.

This is not yet mathematically proven to be the exact cause of the observed circles, but it is now a strong controller-level hypothesis.

---

# 17. RPP Has Also Had Terminal-Heading Oscillation Issues

The previously investigated RPP terminal behavior remains relevant.

Nav2 issue history includes cases where RPP switches between:

```text
rotate to goal heading
        <->
rotate to path heading
```

near a goal.

Localization variation and XY tolerance transitions can cause opposite rotation objectives to alternate.

The project already experienced behavior consistent with this before State Lattice:

```text
small left/right corrections
large rotation away
large rotation back
eventual completion
```

The new State Lattice failure is more severe, but it occurs in the same general controller responsibility:

```text
interpreting and executing orientation-rich path geometry
```

---

# 18. What We Do NOT Conclude Yet

We do **not** currently conclude that:

```text
State Lattice is incorrect
```

because:

- navigation to B succeeded;
- the planner is designed for differential-drive SE(2) planning;
- orientation-aware path generation is still architecturally appropriate for warehouse poses.

We also do **not** conclude that:

```text
the 0.5 m turning radius is wrong
```

or:

```text
rotation_penalty = 5 is wrong
```

because there is not enough evidence yet.

Changing those values now could simply hide the controller's inability to execute a legitimate rotate-in-place or heading transition.

---

# 19. Why We Should Not Remove Rotate-In-Place Behavior

The robot is differential drive.

There are legitimate warehouse cases where an in-place turn is the correct motion.

Example:

```text
robot occupies a parking / waiting pose
goal requires same XY region
but opposite orientation
```

A differential-drive AMR should be capable of:

```text
rotate in place
```

rather than being forced to drive a large loop solely to change heading.

Therefore a long-term solution should not be:

```text
prevent planner from ever generating rotational primitives
```

unless later hardware or safety requirements explicitly demand that.

The controller should be capable of executing legitimate differential-drive orientation changes safely and predictably.

---

# 20. Why Immediate RPP Tuning Was Not Chosen as the Main Direction

One possible next move would be to reduce:

```text
lookahead_dist
min_lookahead_dist
max_lookahead_dist
```

and try to force RPP to follow the lattice more locally.

This may improve the specific behavior.

However, it creates an undesirable engineering tradeoff:

```text
short lookahead
    -> better tight maneuver interpretation

longer lookahead
    -> smoother / more stable aisle tracking
```

The intended warehouse robot needs both:

```text
stable long-path aisle navigation
+
precise orientation-rich terminal maneuvers
```

The project should not depend on discovering one fragile Pure Pursuit lookahead value that barely satisfies both.

RPP tuning remains a possible later comparison / fallback, but it is no longer the preferred next architectural step.

---

# 21. Revised Controller Decision

The next controller to implement will be:

```text
Nav2 MPPI Controller
```

while keeping:

```text
Smac State Lattice
```

as the global planner.

The next architecture is therefore:

```text
Smac State Lattice
        |
        v
MPPI DiffDrive
        |
        v
velocity smoother
        |
        v
Collision Monitor
        |
        v
diff_drive_controller
```

Docking precision control remains separate:

```text
State Lattice + MPPI
        |
        v
reach staging
        |
        v
AprilTag perception
        |
        v
Docking Server precision controller
```

---

# 22. Why MPPI Is a Better Candidate for the New Planner

State Lattice provides a path with meaningful pose orientation.

MPPI is a predictive trajectory controller rather than a simple geometric carrot follower.

For a differential-drive robot it can score candidate trajectories against multiple objectives such as:

```text
path following
path alignment
path orientation
goal position
goal orientation
kinematic constraints
costmap obstacles
```

This is a better conceptual match for:

```text
SE(2)-aware global path
```

than relying only on a Euclidean lookahead point.

Of particular interest is MPPI's ability to use path-angle information from kinematically feasible paths.

The goal is for the local controller to consume the orientation information State Lattice intentionally generated rather than partially reconstructing it from XY geometry.

---

# 23. What Will Remain Unchanged During the MPPI Change

To keep the next change meaningful, the following should remain unchanged initially:

```text
Smac State Lattice
lattice primitive file
global costmap
local costmap
SimpleGoalChecker
velocity smoother
Collision Monitor
MissionManager
RobotExecutor
DockRobot
AprilTag detector
dock_pose_publisher
Docking Server precision controller
```

Only the normal Nav2 local path controller should change:

```text
RPP
  ->
MPPI
```

This gives the next test a clear interpretation.

---

# 24. Next System Test After MPPI

After implementing MPPI, the project should perform a compact mission-level regression rather than many micro-tests.

Recommended sequence:

```text
Home -> B
B -> A
A -> B
B -> Home
```

The most important case remains:

```text
B -> A
```

Expected behavior:

```text
State Lattice produces orientation-aware approach
        |
        v
MPPI executes trajectory without endless circles
        |
        v
staging pose completes
        |
        v
DockRobot transitions to initial perception
        |
        v
AprilTag precision docking
```

---

# 25. Success Criteria for the New Controller

A mission is not considered successful merely because Nav2 eventually returns success.

The controller should demonstrate:

```text
no indefinite circling
no repeated unnecessary ~180° reversals
no continuous left/right heading oscillation
predictable staging entry
stable path following in normal aisles
correct final pose orientation
clean transition into DockRobot precision mode
```

If the controller reaches the goal after a visibly unstable sequence, that should still be considered a navigation-quality failure.

---

# 26. If MPPI Also Fails

If State Lattice + MPPI also produces pathological behavior, the next investigation should focus directly on the path being generated.

At that point inspect:

```text
planned path poses
planned orientations
rotate-in-place segments
primitive transitions
final staging orientation
path updates / replanning
```

Then tune:

```text
rotation_penalty
turning radius
number of headings
reverse expansion
lattice primitive set
analytic expansion behavior
```

Only after path geometry is known to be problematic should the lattice itself be redesigned.

---

# 27. Current Architecture Decision Tree

```text
State Lattice + RPP
        |
        v
ordinary navigation to B works
        |
        v
B -> A causes indefinite circles
        |
        v
controller-path compatibility suspected
        |
        v
keep State Lattice
        |
        v
replace RPP with MPPI
        |
        v
repeat real mission sequence
        |
        +----------------------------+
        |                            |
      works                        fails
        |                            |
        v                            v
MPPI becomes baseline       inspect lattice path itself
```

---

# 28. Updated Status Table

```text
[PASS] docking-only BT experiment removed
[PASS] Docking Server returned to default NavigateToPose BT
[PASS] SmacPlannerLattice loaded as primary planner
[PASS] differential-drive 5 cm lattice integrated
[PASS] lattice asset moved into project configuration
[PASS] ordinary State Lattice + RPP navigation to Station B
[FAIL] B -> A State Lattice + RPP staging
[FAIL] persistent circular motion near staging
[CONFIRMED] failure occurs before AprilTag precision control
[CONFIRMED] failure is not caused by custom docking BT
[LIKELY] local controller / orientation-rich path interaction is central
[NEXT] replace RPP with MPPI DiffDrive
```

---

# 29. Updated Long-Term Navigation Direction

The broader architecture remains unchanged:

```text
Fleet / Mission Manager
        |
        v
semantic resource
        |
        v
route / traffic layer
        |
        v
Smac State Lattice
        |
        v
predictive local controller
        |
        v
terminal pose
        |
        +-------------------+
        |                   |
      normal            precision task
                            |
                       AprilTag dock
```

The important revision is:

```text
previous controller baseline:
RPP

new intended controller baseline to evaluate:
MPPI
```

State Lattice remains the preferred global pose planner.

---

# 30. Main Finding Since the Previous Documentation

The previous project document concluded that:

```text
State Lattice should be tested first
while keeping RPP unchanged
```

That experiment has now been performed.

The result is clear enough to move forward:

> State Lattice improves the planning model by introducing proper differential-drive SE(2) motion, but the existing RPP setup does not reliably execute the resulting orientation-rich path in the difficult B -> A staging case.

The observed failure is severe:

```text
continuous circling
```

rather than merely a small terminal alignment issue.

Therefore the project should now proceed to a controller architecture that is better suited to consuming and optimizing a kinematically feasible pose path.

The next engineering task is:

> **Implement Nav2 MPPI using the differential-drive motion model while retaining the current State Lattice planner and all other navigation / mission / docking layers unchanged.**

---

# 31. Updated Immediate Plan

```text
1. Keep Smac State Lattice as GridBased planner.

2. Keep the current differential-drive lattice:
   warehouse_diff_5cm_r0.5.json

3. Replace RPP with MPPI.

4. Configure MPPI for:
   - differential-drive motion;
   - existing velocity limits;
   - existing acceleration limits;
   - real rectangular footprint / costmap;
   - path orientation awareness;
   - final pose angle tracking.

5. Run:
   Home -> B -> A -> B -> Home

6. If clean:
   adopt State Lattice + MPPI as the navigation baseline.

7. If B -> A still fails:
   inspect the actual lattice path and primitive sequence.

8. Only then tune or regenerate the lattice.
```

---

# 32. Current Technical Baseline

As of this addendum, the project should be understood as:

```text
Global planning:
    Smac State Lattice

Current local controller:
    RPP
    -> demonstrated inadequate in B -> A test

Next local controller:
    MPPI DiffDrive

Mission:
    RobotExecutor + MissionManager

Docking:
    DockRobot
    + AprilTag refinement
    + SimpleNonChargingDock precision controller

Localization:
    AMCL map->odom
    EKF odom->base_footprint

Operational collision prevention:
    costmaps
    controller collision checking
    Collision Monitor

Functional safety:
    remains a separate future industrial safety layer
```

---

# 33. Final Engineering Conclusion

The project has now passed an important architecture checkpoint.

The original problem looked like:

```text
bad final yaw behavior
```

and initially suggested a local controller issue.

Then the project identified a deeper planning issue:

```text
SmacPlanner2D did not encode orientation throughout the path
```

State Lattice corrected that planning limitation.

The first State Lattice system test then exposed a second problem:

```text
RPP does not reliably execute the richer differential-drive path
in the difficult staging scenario
```

This distinction is important.

The current plan is no longer:

```text
tune the old planner/controller combination until it stops misbehaving
```

It is now:

```text
use an orientation-aware feasible planner
+
use a controller designed to evaluate the complete local trajectory
```

The next milestone is therefore **State Lattice + MPPI**.
