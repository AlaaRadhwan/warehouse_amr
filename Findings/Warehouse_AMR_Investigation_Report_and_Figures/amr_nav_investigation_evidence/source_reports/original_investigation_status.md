# Warehouse AMR Navigation Investigation — Current Findings and Next Direction

**Project:** MTS_AMR / `warehouse_amr`  
**Date:** 2026-10-20  
**Scope:** Navigation failure investigation around Smac State Lattice + MPPI, especially terminal orbiting, looping global plans, and failure to converge to final yaw.

---

## 1. Purpose of this document

This document captures the current state of the navigation investigation so that we do not lose the reasoning, experiments, eliminated hypotheses, or the exact direction of the next work.

The recurring symptom is that the AMR can navigate most of a route normally, but close to a goal it may:

- begin orbiting or turning repeatedly,
- receive a global plan containing a large loop/ring,
- reach the correct XY position but fail to converge to the requested yaw,
- continue rotating or become nearly motionless,
- fail to trigger recovery promptly,
- and sometimes show the same behavior even on an ordinary Home → Station B goal.

The investigation has now produced enough evidence to separate several planner, controller, and command-pipeline effects.

---

## 2. Current navigation architecture

The intended current navigation stack is:

```text
Mission / RobotExecutor
        ↓
NavigateToPose / DockRobot
        ↓
Smac State Lattice
        ↓
MPPI (DiffDrive)
        ↓
velocity_smoother
        ↓
Collision Monitor
        ↓
diff_drive_controller
        ↓
Gazebo / later real motor interface
```

For docking:

```text
State Lattice + MPPI
        ↓
reach staging pose
        ↓
DockRobot precision phase
        ↓
AprilTag / detected_dock_pose
        ↓
DockingServer precision controller
```

The precision docking controller is not intended to solve ordinary Nav2 path-following problems.

---

## 3. Important robot/navigation facts

Robot:

- Differential drive.
- Approx. 1.0 m long × 0.60 m wide.
- Navigation footprint approximately:

```yaml
[[0.50, 0.37], [0.50, -0.37], [-0.50, -0.37], [-0.50, 0.37]]
```

- Wheel separation ≈ 0.69 m.
- Wheel radius ≈ 0.10 m.

State Lattice:

- 0.05 m resolution.
- Differential-drive primitive set.
- 16 headings.
- Nominal curved primitive radius ≈ 0.5 m.
- Rotate-in-place primitives exist.
- Reverse expansion is currently disabled.
- Project lattice is based on the Nav2 differential sample primitive set.

Important point:

> The 0.5 m primitive turning radius does **not** mean the physical robot cannot rotate in place. The lattice explicitly contains rotate-in-place primitives.

---

## 4. Symptom history

### 4.1 Smac2D + RPP

Earlier configuration:

```text
SmacPlanner2D + Regulated Pure Pursuit
```

Observed:

- ordinary route execution could be smooth,
- but terminal heading behavior sometimes oscillated,
- Station B → Station A could produce large terminal rotations before docking.

This initially looked like a controller issue.

---

### 4.2 State Lattice + RPP

We then changed the global planner to:

```text
Smac State Lattice + RPP
```

Observed:

- Station B could be reached normally,
- B → A could enter repeated / indefinite turning while still in the staging navigation phase,
- the AprilTag precision controller had not yet taken over.

Conclusion:

> The failure was not caused by AprilTag precision docking.

---

### 4.3 State Lattice + MPPI

We then changed to:

```text
Smac State Lattice + MPPI DiffDrive
```

Observed:

- terminal orbiting still occurred,
- the visible global plan sometimes looked like a ring,
- the robot sometimes appeared not to follow the visible global path,
- the same basic failure could appear on ordinary navigation goals as well.

This moved the investigation away from docking-specific logic.

---

## 5. Raw planner path finding: the ring is not created by the smoother

A second RViz path display was added for:

```text
/unsmoothed_plan
```

The red raw State Lattice plan itself already contained the strange loop/ring.

Therefore:

```text
State Lattice search / terminal expansion
        ↓
already contains the loop
        ↓
Smac smoother
        ↓
final /plan
```

The built-in Smac smoother is therefore **not the origin of the ring**.

It may still alter geometry, but it is not required for the failure to appear.

---

## 6. Planner parameter experiments that did not solve the problem

### 6.1 Analytic expansion maximum length

Changed:

```yaml
analytic_expansion_max_length: 3.0
```

to:

```yaml
analytic_expansion_max_length: 2.0
```

Result:

- no meaningful change,
- ordinary Home → B could still show the same abnormal behavior.

Conclusion:

> Merely shortening analytic expansion does not solve the problem.

---

### 6.2 Rotation penalty

Changed:

```yaml
rotation_penalty: 5.0
```

to:

```yaml
rotation_penalty: 2.0
```

Reason:

- a high rotation penalty may make the lattice prefer long forward loops rather than an in-place rotation.

Result:

- the failure still occurred.

Conclusion:

> Rotation cost may influence the exact planner solution, but it is not by itself the root fix.

---

## 7. AprilTag / docking is not the root cause

The same abnormal navigation behavior persisted when:

- the robot was sent to Home,
- AprilTag perception was stopped,
- ordinary navigation goals were used.

Therefore:

> AprilTag perception and the precision docking controller are not the root cause of the general orbiting / terminal deadlock.

AprilTag may still be active later in the DockRobot state machine, but it does not explain the Home → B or pre-staging failures.

---

## 8. Command timestamp issue that was found

Earlier terminal logs showed:

```text
diff_drive_controller:
Ignoring the received message ...
because it is older than the current time by ~0.8–0.98 s
allowed timeout = 0.5 s
```

The velocity chain uses `TwistStamped`.

The velocity smoother was originally preserving old incoming timestamps. The following was added:

```yaml
velocity_smoother:
  ros__parameters:
    stamp_smoothed_velocity_with_smoothing_time: true
```

Later rosbag analysis showed:

- the stale-message problem was no longer the dominant observed failure,
- MPPI itself was producing the problematic command behavior,
- smoother / Collision Monitor were generally forwarding those commands rather than inventing the orbit.

Conclusion:

> The stale timestamp issue was real and needed fixing, but it does not explain the remaining terminal orbit/deadlock.

---

## 9. Gazebo physics rate

The warehouse world inherited:

```xml
<max_step_size>0.0005</max_step_size>
```

which corresponds to:

```text
2000 Hz physics
```

This appears to be a leftover from an earlier high-rate dynamics workflow.

For a differential-drive AMR with a 100 Hz controller manager, 2 kHz is unnecessarily expensive.

A more reasonable target is:

```text
500–1000 Hz physics
```

for example:

```xml
<max_step_size>0.001</max_step_size>   <!-- 1000 Hz -->
```

or later:

```xml
<max_step_size>0.002</max_step_size>   <!-- 500 Hz -->
```

Important distinction:

> A high physics rate may contribute to CPU load, delayed callbacks, missed controller deadlines, or timing instability, but it cannot explain why `/unsmoothed_plan` itself contains a large ring.

---

## 10. Why reverse is still disabled for now

Current planner:

```yaml
allow_reverse_expansion: false
```

Current MPPI:

```yaml
vx_min: 0.0
```

This is intentional **during diagnosis**, not a final product decision.

If only the planner were allowed to generate reverse segments while MPPI remained forward-only, the planner/controller pair would become inconsistent.

Reverse should later be introduced as one coordinated milestone:

```text
State Lattice:
    allow_reverse_expansion = true

MPPI:
    vx_min < 0
    enforce_path_inversion = true

plus:
    inversion tolerances
    mission-level reverse policy
    safety validation
```

For the commercial AMR, reverse is expected to be useful and probably necessary in constrained warehouses.

---

# 11. Rosbag investigation results

## 11.1 Original B → A failure

The rosbag showed:

- normal navigation for much of the route,
- excessive turning / orbiting near the staging goal,
- MPPI itself issuing the rotation commands,
- no evidence that velocity_smoother or Collision Monitor was creating the orbit,
- no major localization jump explaining it.

This made the MPPI / planner interaction much more important than the downstream drive interface.

---

## 11.2 Artificial yaw-π experiment

To remove the difficult terminal geometry, Station A staging was temporarily tested with:

```text
same staging XY
yaw = 180°
```

instead of the normal yaw = 0°.

The idea was:

```text
normal B → A staging:
arrival direction ≈ west
required final heading = east

yaw-π test:
arrival direction ≈ west
required final heading = west
```

If the U-turn geometry were the root cause, this run should have succeeded cleanly.

It did **not**.

### What happened

The approach was initially good.

Then near the goal:

- the robot passed very close to the goal XY,
- yaw was still outside tolerance,
- after the robot passed / ended up with the goal slightly behind it,
- State Lattice produced a roughly 3 m loop,
- MPPI later entered a near-zero / deadlocked command state.

This disproved the earlier hypothesis that the opposite terminal heading was the sole cause.

Conclusion:

> The U-turn geometry is a trigger/amplifier, not the fundamental cause.

---

## 11.3 Goal tolerance interaction

Current goal checker is approximately:

```yaml
xy_goal_tolerance: 0.10
yaw_goal_tolerance: 0.05
```

That yaw tolerance is only about:

```text
2.9°
```

The robot can therefore be only a few centimetres from the target XY and still not succeed because its yaw is outside tolerance.

Once the target position falls slightly behind the robot, the forward-only planner/controller has a much harder state to recover from.

This does not necessarily mean the tolerance is "wrong," but it strongly amplifies the failure.

---

## 11.4 PathFollowCritic threshold experiment

Original:

```yaml
PathFollowCritic:
  threshold_to_consider: 1.4
```

Changed to:

```yaml
PathFollowCritic:
  threshold_to_consider: 0.10
```

while keeping:

```yaml
GoalCritic:
  threshold_to_consider: 1.4
```

### Improvement

The approach improved substantially:

- velocity decayed smoothly,
- overshoot disappeared,
- the robot stopped about 3–4 cm from the goal,
- arrival yaw error improved from roughly 25–30° to about 11°,
- the earlier CCW drift during approach disappeared.

This is meaningful evidence that dropping path-follow guidance at 1.4 m was hurting the approach.

### Failure that remained

Inside the final XY tolerance:

- the planner still produced large loops when the requested heading was not satisfied,
- MPPI failed to converge cleanly to goal yaw,
- angular command behavior became weak/noisy,
- the robot eventually idled / deadlocked.

Conclusion:

> `PathFollowCritic.threshold_to_consider = 0.10` improves approach quality but does not solve the terminal heading problem.

---

# 12. MPPI critic gating near the goal

The current MPPI critic layout creates several distance-based behavior transitions.

Roughly:

```text
far from goal:
    path-following critics dominate

closer:
    GoalCritic becomes active

very close:
    some path critics disable
    GoalAngleCritic becomes active
```

The original 1.4 m PathFollow handoff was clearly too aggressive for this setup.

However, the latest run shows that the remaining deadlock happens even closer to the goal, where the controller must solve final heading alignment while the planner may be presenting a looped path.

So the controller issue is no longer best described as simply:

```text
"MPPI breaks inside 1.4 m"
```

A better current description is:

> MPPI handles the approach much better when path-following remains active, but it still has difficulty when the robot is very close to the endpoint and the State Lattice path folds/loops around the robot to satisfy final orientation.

---

# 13. Progress checker finding

Current progress checker is based on `PoseProgressChecker` and uses both translation and angle.

Conceptually, progress is accepted if either:

```text
translation > required_movement_radius
OR
rotation > required_movement_angle
```

Therefore, a robot that is mostly stuck but rotates enough every few seconds can keep satisfying the progress condition.

This helps explain why recovery may not fire promptly during pathological orbiting.

Also important:

> Ordinary new path updates do **not** automatically reset the progress checker unless the selected progress-checker plugin itself changes.

So repeated replanning is not a valid explanation for progress-checker reset by itself.

---

# 14. Similar online projects / external references

The online comparison has become highly valuable because one public project reproduced a failure that is strikingly similar to ours.

---

## 14.1 `glebobos/relobot` — strongest relevant reference

This project uses:

```text
SmacPlannerLattice
+
MPPI
+
differential-drive primitives
```

and has detailed recorded navigation diagnostics.

Most importantly, its documentation contains a **goal-loop failure** very similar to ours.

They recorded cases where:

```text
direct goal distance ≈ 0.25–0.28 m
global lattice path ≈ 3.0 m
signed turn ≈ ±2π
arc radius ≈ 0.5 m
```

That is extremely close to our symptom:

```text
robot almost at goal
        ↓
planner creates ~3 m ring
```

They also found:

- rings persisted with analytic expansion disabled,
- reducing `rotation_penalty` alone did not eliminate the loop,
- therefore neither analytic expansion nor rotation penalty alone explains the phenomenon.

This independently supports our own findings.

### Very important relobot finding: MPPI path association

Their later diagnosis found a different problem in MPPI's path handler:

```text
loop / U-shaped global path
        ↓
multiple portions of the path are geometrically close to robot
        ↓
bounded nearest-pose search associates robot with the wrong / early branch
        ↓
transformed local path ends at an intermediate location
        ↓
MPPI thinks it is close to its local endpoint
        ↓
goal critics activate
path critics deactivate
        ↓
controller stalls / emits near-zero commands
```

Their test changed:

```yaml
max_robot_pose_search_dist: 1.0
```

to:

```yaml
max_robot_pose_search_dist: 2.5
```

for ordinary navigation.

That removed their reproduced short U-turn / intermediate-local-goal stall in the tested scenario.

This does **not** prove that our bug is identical, but it is currently the closest public analogue we have found.

### Their current ordinary MPPI configuration includes

```yaml
motion_model: DiffDrive
prune_distance: 1.6
max_robot_pose_search_dist: 2.5

PathAlignCritic:
  offset_from_furthest: 8
  use_path_orientations: false

PathFollowCritic:
  offset_from_furthest: 6

GoalCritic:
  threshold_to_consider: 1.0

GoalAngleCritic:
  threshold_to_consider: 0.25
```

This is interesting because our configuration previously used:

```yaml
PathAlignCritic:
  offset_from_furthest: 20
  use_path_orientations: true
```

which is substantially different.

The relobot project also documents that path-follow threshold changes alone did not solve every stall, which matches our latest result.

---

## 14.2 `ArianShafagh/autonomous-industrial-mobility` / RoboFetch

This ROS 2 Jazzy project is also useful.

It loads and tests:

```text
SmacPlannerLattice
+
MPPI DiffDrive
```

with the same Nav2 differential 0.5 m lattice primitive set.

Its MPPI configuration is very close to standard Nav2:

```yaml
GoalCritic.threshold_to_consider: 1.4
GoalAngleCritic.threshold_to_consider: 0.5

PathAlignCritic:
  offset_from_furthest: 20
  use_path_orientations: false

PathFollowCritic:
  offset_from_furthest: 5
  threshold_to_consider: 1.4
```

Important differences from us:

- `use_path_orientations: false`
- reverse motion is allowed in MPPI (`vx_min < 0`)
- goal tolerances are much looser:

```yaml
xy_goal_tolerance: 0.25
yaw_goal_tolerance: 0.25
```

The project explicitly notes that tightening position tolerance to roughly 0.12–0.20 m caused MPPI navigation failures in their setup.

This is not direct proof for our robot, but it supports the idea that demanding very tight ordinary Nav2 terminal tolerances can be counterproductive.

For docking, this also supports separating:

```text
ordinary navigation tolerance
```

from:

```text
precision docking tolerance
```

rather than asking generic NavigateToPose to perform final centimetre-level precision.

---

## 14.3 `IndusNav-AMR`

Another AMR reference uses:

```text
MPPI DiffDrive
+
NavFn
```

and also uses approximately:

```yaml
xy_goal_tolerance: 0.25
yaw_goal_tolerance: 0.25
```

This project is less directly comparable because its global planner is NavFn rather than State Lattice, but it reinforces that 0.25 m / 0.25 rad is a common practical Nav2 starting point rather than 0.10 m / 0.05 rad.

---

# 15. Current hypothesis ranking

## HIGH — State Lattice terminal loop generation

We have repeatedly observed:

```text
robot near goal
        ↓
raw /unsmoothed_plan becomes ~3 m loop
```

This is planner-side behavior.

It occurs before smoothing.

A very similar public project reproduced the same pattern.

---

## HIGH — MPPI path association / transformed-plan handling on looped paths

This is now one of the strongest hypotheses.

MPPI does not use the entire global plan directly. Its path handler:

1. finds the closest pose on the global plan within a bounded integrated-distance search window,
2. prunes from that point,
3. transforms only a limited portion of the path.

When a loop passes close to the robot more than once, nearest-pose association can become ambiguous.

A public project with the same planner/controller combination documented exactly this kind of failure.

This may explain why:

- the global path looks plausible enough,
- MPPI then produces a strange local target,
- terminal commands collapse toward zero or become inconsistent.

---

## MEDIUM-HIGH — Near-goal MPPI critic handoff

Evidence:

- lowering `PathFollowCritic.threshold_to_consider` from 1.4 to 0.10 clearly improved the approach,
- but the final deadlock remained.

So critic gating is a contributor, but no longer looks sufficient as a complete explanation.

---

## MEDIUM-HIGH — Tight terminal tolerances amplify the failure

Current:

```yaml
xy_goal_tolerance: 0.10
yaw_goal_tolerance: 0.05
```

These tolerances make a small terminal yaw error enough to keep the action alive.

Once the robot is only a few centimetres away but not correctly aligned, State Lattice may generate an unnecessarily large loop.

For precision docking stations, demanding this precision from generic NavigateToPose is probably unnecessary.

---

## MEDIUM — Continuous 1 Hz replanning

Repeated replanning can replace a reasonable plan with a pathological near-goal loop.

However, current evidence does not show it is the sole cause.

It is more likely an amplifier:

```text
near-goal state
        ↓
new planner solution
        ↓
looped path
        ↓
MPPI association / terminal controller problem
```

---

## LOW / effectively ruled out as primary causes

### AprilTag precision docking

Not responsible for ordinary Home → B failures.

### Velocity smoother / Collision Monitor

They do not appear to be inventing the terminal orbit.

### Major localization jump

No strong evidence that localization is jumping enough to create the failure.

### Smac smoother

The raw `/unsmoothed_plan` already contains the ring.

### Analytic expansion max length alone

3.0 → 2.0 did not solve it.

### Rotation penalty alone

5.0 → 2.0 did not solve it.

---

# 16. What we are doing now

The investigation should now stop random critic tuning and directly inspect **the path that MPPI itself sees**.

The most useful next diagnostic is:

```yaml
FollowPath:
  visualize: true
```

and record:

```text
/plan
/unsmoothed_plan
/transformed_global_plan
/trajectories
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel
/odom
/tf
/tf_static
/clock
```

The critical question is:

```text
global State Lattice path
        ↓
MPPI PathHandler
        ↓
/transformed_global_plan
```

Does `/transformed_global_plan` still represent the correct branch of the loop?

Or does it:

- associate the robot with the wrong part of the loop,
- crop the path to an intermediate endpoint near the robot,
- make MPPI think it has reached a local target even though the action goal is still not satisfied?

This is now more informative than another arbitrary critic-weight change.

---

# 17. Proposed next controlled experiment

Keep the latest useful approach setting:

```yaml
PathFollowCritic:
  threshold_to_consider: 0.10
```

Enable:

```yaml
FollowPath:
  visualize: true
```

Do **not** change multiple other parameters yet.

Record a failing run including:

```text
/transformed_global_plan
/trajectories
```

Then compare, during the final deadlock:

1. robot pose,
2. full `/plan`,
3. `/unsmoothed_plan`,
4. `/transformed_global_plan`,
5. final point of `/transformed_global_plan`,
6. true NavigateToPose endpoint,
7. MPPI optimal trajectory,
8. `cmd_vel_nav`.

If the transformed local plan ends near the robot while the true global endpoint remains elsewhere along the loop, we will have strong evidence for the same path-association mechanism seen in `relobot`.

Only after that should we test:

```yaml
max_robot_pose_search_dist
```

systematically.

We should **not** blindly copy 2.5 m without first checking our local-costmap size, loop geometry, and actual transformed plan.

---

# 18. GoalCritic 0.10 proposal

Claude proposed:

```yaml
GoalCritic:
  threshold_to_consider: 0.10

PathFollowCritic:
  threshold_to_consider: 0.10
```

This is still a valid diagnostic.

Its purpose is:

> Determine whether the goal-distance critic is masking / competing with final yaw behavior inside the terminal region.

However, after finding the very similar `relobot` failure, the path-handler / transformed-plan diagnosis is now at least as important.

Recommended order:

```text
1. Observe /transformed_global_plan + trajectories
2. Determine whether path association is wrong
3. If association is correct, run GoalCritic threshold = 0.10
4. Then tune terminal critics only with evidence
```

---

# 19. Product-level direction after the bug is fixed

The long-term architecture should still remain:

```text
Mission / Fleet
        ↓
Semantic resource layer
        ↓
Route / traffic layer
        ↓
State Lattice global feasibility
        ↓
MPPI local predictive control
        ↓
velocity conditioning
        ↓
Collision Monitor
        ↓
PLC / drive interface
        ↓
independent safety layer
```

Precision docking remains:

```text
Nav2 reaches staging region
        ↓
DockRobot
        ↓
AprilTag precision phase
```

Long-term improvements expected:

- coordinated reverse support,
- separate goal-checker policies for ordinary navigation vs docking staging vs parking,
- less aggressive reliance on generic NavigateToPose for centimetre-level precision,
- route/traffic semantics later,
- independent certified safety separate from Nav2 Collision Monitor.

---

# 20. Current "do not do" list

Until the current terminal failure is understood:

- Do not randomly tune many MPPI critic weights together.
- Do not disable collision checking.
- Do not loosen all goal tolerances globally just to hide the bug.
- Do not change station yaw merely to avoid the failure.
- Do not blame AprilTag for ordinary navigation failures.
- Do not reintroduce a recovery-spin theory without evidence.
- Do not assume the Smac smoother created the ring.
- Do not enable planner reverse without enabling compatible controller reverse.
- Do not abandon State Lattice solely because RPP or MPPI currently fails at the terminal.
- Do not treat the current warehouse geometry as the only target environment.

---

# 21. Current concise diagnosis

The best current summary is:

> The AMR usually follows State Lattice paths well during cruise. Close to the goal, State Lattice can generate a large loop/ring to satisfy a small remaining heading error. MPPI's near-goal critic behavior contributes to poor terminal convergence, and the latest PathFollow threshold experiment improved the approach but did not remove the final deadlock. A highly similar public State-Lattice + MPPI project found that looped/U-shaped paths can confuse MPPI's bounded nearest-path association and cause its transformed local plan to terminate at the wrong intermediate location. Our next step is therefore to record and inspect `/transformed_global_plan` and MPPI trajectories before making further controller changes.

That is the current investigation baseline.
