# Documentation index

Warehouse AMR is an independent ROS 2 autonomous mobile robot project, developed **in simulation only** (ROS 2 Jazzy, Nav2 1.3.12,
Gazebo). Start here; the repository root [`README.md`](../README.md) gives the overview and launch instructions.

## 1. Overview and architecture
| Document | Contents |
|---|---|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | Packages, launch composition, data flow, what runs in the one-command launch and what does not. |

## 2. Current accepted navigation baseline
| Document | Contents |
|---|---|
| [`NAVIGATION_BASELINE.md`](NAVIGATION_BASELINE.md) | NavFn + 2D-path MPPI + PreferForwardCritic + Nav2 Route Server: configuration, rationale, behavior, limitations. Accepted for continued **simulation** development only. |
| [`STRUCTURED_ROUTING.md`](STRUCTURED_ROUTING.md) | Route Server, route graph (provisional fixture) and the route behavior tree. |

## 3. Validation results (single runs, simulation)
| Document | Contents |
|---|---|
| [`CONSOLIDATED_VALIDATION.md`](CONSOLIDATED_VALIDATION.md) | Four-test acceptance run, including the routed goal-yaw fix. Scripts: `investigations/consolidated_validation_2026-10-29/`. |
| [`PREFER_FORWARD_CRITIC_EVALUATION.md`](PREFER_FORWARD_CRITIC_EVALUATION.md) | Reverse-motion preference tests. |
| [`NAVFN_MPPI_EVALUATION.md`](NAVFN_MPPI_EVALUATION.md), [`LATTICE_2D_FORWARD_EVALUATION.md`](LATTICE_2D_FORWARD_EVALUATION.md) | Planner/controller comparisons that led to the baseline. |
| [`figures/`](figures/) | Trajectory figures used by the reports. |

## 4. Historical investigations and experiments
| Location | Contents |
|---|---|
| [`../Findings/README.md`](../Findings/README.md) | Dated reports (2026-10-14 to 10-25), labelled with their current status. |
| [`../calibration/`](../calibration/) | Yaw-rate system identification, wheel-separation calibration (0.92754, simulator-specific), B→A validation harness. |
| [`../debug/`](../debug/) | Pre-calibration spin analysis and runtime audit. |
| [`../investigations/`](../investigations/) | Instrumented B→A variability study; NavFn/MPPI/lattice evaluations; structured routing; consolidated validation. |
| [`DATA_MANIFEST.md`](DATA_MANIFEST.md) | Raw rosbags and large derived data kept **out of Git** (paths, sizes, SHA-256). |
| Branch `feature/rotation-shim-integration` | Unfinished, **experimental** Rotation Shim evaluation. Not part of the baseline. |

## 5. Limitations and future work
| Document | Contents |
|---|---|
| [`LIMITATIONS_AND_FUTURE_WORK.md`](LIMITATIONS_AND_FUTURE_WORK.md) | What is and is not established, known open issues, planned work. |
| [`../THIRD_PARTY_REVIEW.md`](../THIRD_PARTY_REVIEW.md) | Third-party assets, licences and unresolved ownership questions. |
