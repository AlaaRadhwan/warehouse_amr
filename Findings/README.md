# Findings: historical investigation reports

These are dated engineering records written during development. **They are historical.** Each reflects what was known on its date
and is kept unedited (apart from the project-name header, see below) so the reasoning behind later decisions can be followed.
For the **current** accepted configuration and its evidence start at [`docs/README.md`](../docs/README.md).

**Naming note.** Reports written before 2026-11 carry the earlier working name `MTS_AMR` in a header line. The project is now called
**Warehouse AMR**; the header lines say so, the technical content was not changed, and no institutional affiliation is implied.
One report file was renamed for this reason (`git log --follow` shows its history).

## Reading order

| Date | Report | What it is | Status today |
|---|---|---|---|
| 2026-10-14 | [`Warehouse_AMR_Comprehensive_Project_Documentation.md`](Warehouse_AMR_Comprehensive_Project_Documentation.md) | Whole-project snapshot at commit `1460884`: capability table, docking, AprilTag. | **Superseded.** Describes the Regulated Pure Pursuit era; its capability table is not current. |
| 2026-10-18 | [`…_Comprehensive_Project_Documentation_2026-10-18.md`](Warehouse_AMR_Comprehensive_Project_Documentation_2026-10-18.md) | Next snapshot at `main` commit `fe42d1a`. | **Superseded** by later reports; overlaps the file above. |
| 2026-10-19 | [`…_Navigation_Architecture_Decision_and_Plan_2026-10-19.md`](Warehouse_AMR_Navigation_Architecture_Decision_and_Plan_2026-10-19.md) | Decision record: Smac State Lattice + MPPI as target baseline. | **Superseded** for the baseline choice: the accepted simulation baseline is NavFn + 2D-path MPPI + PreferForwardCritic + Route Server. Its design principles remain useful. |
| 2026-10-19 | [`…_Navigation_Progress_Addendum_2026-10-19.md`](Warehouse_AMR_Navigation_Progress_Addendum_2026-10-19.md) | Findings after the 10-18 snapshot (RPP vs lattice, path/pose mismatch). | Historical. |
| 2026-10-20 | [`…_Navigation_Investigation_Status_2026-10-20.md`](Warehouse_AMR_Navigation_Investigation_Status_2026-10-20.md) | Terminal orbiting / looping plans / final-yaw investigation. | Historical. Hypotheses in it were later tested; see the 10-25 record. |
| 2026-10-22 | [`Warehouse_AMR_Investigation_Report_and_Figures/`](Warehouse_AMR_Investigation_Report_and_Figures/amr_nav_investigation_evidence/README.md) | Terminal navigation investigation with figures, scripts and summary data. | Historical evidence package. |
| 2026-10-23 | [`Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23/`](Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23/Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23.md) | B→A spin/stall variability (small sample). | Historical. Its header names a branch/commit that is not the current baseline. |
| 2026-10-25 | [`…_Navigation_Investigation_Lessons_and_Rotation_Shim_Decision_2026-10-25.md`](Warehouse_AMR_Navigation_Investigation_Lessons_and_Rotation_Shim_Decision_2026-10-25.md) | Consolidated lessons and the decision to evaluate a Rotation Shim. | Historical; the shim evaluation is on a separate experimental branch and is **not** part of the accepted baseline. |

## Related evidence outside this folder

| Location | Contents |
|---|---|
| [`calibration/yaw_rate_sysid/`](../calibration/yaw_rate_sysid/) | Yaw-rate system identification and the simulator wheel-separation calibration (0.92754). |
| [`calibration/nav_validation/`](../calibration/nav_validation/) | B→A validation harness and run records (including a labelled INVALID run). |
| [`debug/`](../debug/) | Pre-calibration spin analysis and runtime audit. |
| [`investigations/`](../investigations/) | Instrumented variability study, NavFn/MPPI/lattice evaluations, structured routing and the consolidated validation. |

Single runs and small samples in these reports do not establish statistical reliability; this is a simulation-only project.
