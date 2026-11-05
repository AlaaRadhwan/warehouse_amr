# Case study 2: planner / MPPI evaluation and reverse-navigation policy

*Simulation only; single runs (n = 1 per condition); no reliability claim.*

## Problem
The robot is bidirectional, but a navigation stack that treats forward and reverse as equals can drive an entire route backwards, and one that
forbids reverse spins awkwardly in tight spots. The goal was a controller/planner combination that normally drives forward, keeps to a
route lane, and can still back out of a tight pocket.

## Investigation
Planner/controller combinations were compared on the same scenarios using Gazebo ground truth: State Lattice and NavFn global planners,
MPPI with path-orientation terms on or off, reverse allowed, forward-only (`vx_min 0`), and a soft `PreferForwardCritic`.
Source reading of Nav2 1.3.12 gave a mechanism: NavFn paths carry yaw 0 on every pose except the last, so MPPI's path-orientation terms
(`PathAlignCritic.use_path_orientations`, `PathAngleCritic` mode 2) pull the robot toward heading east and make westward paths look like reverse paths.
Those terms were therefore switched off for NavFn ("2D-path interpretation").

## Measured evidence (ground truth, routed B→A staging unless stated)
| Configuration | Duration | Reverse distance | Max lane deviation |
|---|---|---|---|
| 2D paths, reverse allowed, no preference | 66.1 s | 18.05 m (whole route) | 0.48 m |
| 2D paths, forward-only | 47.7 s | 0 m | 0.13 m |
| **2D paths + PreferForwardCritic (weight 5.0)** | **49.0 s** | **0.24 m** | **0.15 m** |

Free B→Home with the critic: 46.3 s, 0.62 m reverse (forward-only prior: 44.6 s). Back out of a pillar pocket (synthetic start, 1.64 m needed):
14.3 s with the critic, versus 21.9 s and 277° of in-place rotation forward-only, with a start about 6 cm roomier, so this is a benefit not a proof of necessity.
Initial attempts with path orientations left on drifted 1.71 m off the lane and reversed about 9 m.

## Solution (accepted simulation baseline)
NavFn + MPPI with `use_path_orientations false`, `PathAngleCritic.mode 0`, `enforce_path_inversion false`, `vx_min -0.35`, and
`PreferForwardCritic {weight 5.0, power 1, threshold 0.5}`, selected with
`planner:=navfn mppi_path_interpretation:=2d_prefer_forward`. The critic prices reverse distance linearly; it does not forbid it.

## Validation
The consolidated validation re-ran four scenarios on this configuration: routed B→A (48.4 s, 0.24 m reverse), free B→Home (48.2 s, 0.71 m reverse),
a rear-obstacle test and a routed 90° goal. All completed with no stall, loop or oscillation.

## Limitations
* Reverse remains a soft preference: expect short back-out arcs (0.1-0.7 m seen); a bounded undock maneuver would be needed to guarantee behavior.
* State Lattice with reverse expansion matched NavFn within single-run noise, so NavFn was chosen as simpler, not proven better.
* The critic's cost scale was not compared with other critics; weight 5.0 was chosen from these runs, not derived.
* In the rear-obstacle test the closest footprint margin was **0.05 m** at the goal. This is not evidence of reliable or hardware-safe
  obstacle avoidance; low obstacles outside the LiDAR plane are not covered.

## Detailed reports
[`docs/NAVFN_MPPI_EVALUATION.md`](../NAVFN_MPPI_EVALUATION.md), [`docs/PREFER_FORWARD_CRITIC_EVALUATION.md`](../PREFER_FORWARD_CRITIC_EVALUATION.md),
[`docs/LATTICE_2D_FORWARD_EVALUATION.md`](../LATTICE_2D_FORWARD_EVALUATION.md), [`docs/CONSOLIDATED_VALIDATION.md`](../CONSOLIDATED_VALIDATION.md),
[`docs/NAVIGATION_BASELINE.md`](../NAVIGATION_BASELINE.md).

*Rotation Shim, an alternative terminal-rotation controller, was tried separately on an unfinished experimental branch and is not part of this baseline.*
