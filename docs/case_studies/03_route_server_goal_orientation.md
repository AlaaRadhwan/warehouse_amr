# Case study 3: Route Server last-mile goal-orientation failure and correction

*Simulation only; single run for the verification.*

## Problem
With the Nav2 Route Server, a request to arrive at Station B with a yaw of 90° ended with Nav2 reporting **SUCCEEDED** while the robot
faced 0°: a wrong-orientation arrival reported as success.

## Investigation
The goal checker and MPPI's goal terms compare against the **last pose of the path**. A Route Server path ends with the heading of its last
graph edge (0° at B), so the requested yaw never reached the controller. The route behavior tree skipped its "last mile" planner when the
goal was within 0.1 m of the route end, because that test (`ArePosesNear`) compares position only. When the goal sat on the route end, the
requested yaw was silently replaced. Before the fix the robot ended 90.1° off the requested yaw.

## Solution
One block in `behavior_trees/navigate_on_warehouse_route.xml`: the last mile is always planned when the goal lies within 1.5 m of the
route end, including the same position, starting from the route's end pose and concatenated to the route. The planner's path ends with
the requested pose. The 1.5 m guard still fails the tree instead of allowing a free-space shortcut. No tolerance or critic value changed.

## Validation (routed Home→B, requested yaw +90°)
| Measure | Value |
|---|---|
| Action result | SUCCEEDED, after the stopped-goal check |
| Final yaw error vs requested | -3.5° (0.061 rad; tolerance 0.20 rad) |
| Final position error | 0.041 m (tolerance 0.25 m) |
| Last-mile plan | 2 poses, final yaw 90.0° |
| Duration / max lane deviation | 39.8 s / 0.16 m |

## Limitations
* One run. A deliberately unreachable yaw was **not** tested; the checker now compares with the requested pose by construction only.
* Route Server snaps start and goal to the nearest graph node with no distance limit; the BT's 1.5 m guard bounds the last mile.
* The route graph is a provisional 6-node fixture, so this shows the mechanism, not an approved route network.
* The case concerns the fix of a specific defect in this project's tree, not a bug claim about Nav2.

## Detailed reports
[`docs/CONSOLIDATED_VALIDATION.md`](../CONSOLIDATED_VALIDATION.md) (section 2), [`docs/STRUCTURED_ROUTING.md`](../STRUCTURED_ROUTING.md),
[`docs/NAVIGATION_BASELINE.md`](../NAVIGATION_BASELINE.md).
