# MTS_AMR — Navigation Architecture Experiments, Structured Routing, and Development-Baseline Decision

**Project:** MTS_AMR / `warehouse_amr`  
**Coverage:** 26–29 October 2026  
**Status date:** 29 October 2026  
**Preceding record:** `MTS_AMR_Navigation_Investigation_Lessons_and_Rotation_Shim_Decision_2026-10-25.md`  
**System:** ROS 2 Jazzy; Nav2 1.3.12; Gazebo; differential-drive AMR; NavFn / Smac State Lattice; MPPI; Nav2 Route Server  
**Purpose:** A technical continuation record for engineering development, reproducibility, future reviews, and potential article preparation. This is **not** a statistical reliability, hardware-safety, or production-readiness report.

> **Executive conclusion (29 October).** We provisionally accepted **NavFn + MPPI configured for geometric (“2D”) paths + `PreferForwardCritic` at weight 5.0 + reverse capability + Nav2 Route Server** as the **working simulation development architecture**. The most decisive discovery was not that NavFn inherently outperformed State Lattice: in routed trials, the global connector planner was often never called. Instead, **MPPI was interpreting travel-direction headings as vehicle headings and was permitted to choose long reverse journeys**. Correcting the path interpretation and adding a forward preference reduced the troublesome B → A staging departure from **1.71 m** maximum route deviation and **9.1–9.2 m** of reverse travel under the original routed configuration to **0.14 m** and **0.24 m** in the final validation. We also corrected a **false-success goal-yaw bug** in the route Behavior Tree: a 90° requested heading now ended within **3.5°**, rather than being silently replaced by the last route edge's 0° heading. The system is suitable for continued simulation development, **not validated for unattended operation**. The provisional graph is not approved; the project owner will design the actual warehouse routes.

---

## 1. Relationship to the October 25 investigation

The October 25 record covered the earlier terminal loops, angular saturation, calibrated stall, and independent differential-drive system identification. It concluded that:

- The original Gazebo plant had a measured actual/commanded yaw-rate ratio of approximately **1.07812**. The effective wheel-separation correction `wheel_separation_multiplier: 0.92754` corrected that measured steady-state yaw discrepancy in the simulated plant.
- This calibration **did not prove navigation reliability**. An uncalibrated robot sometimes completed the same mission, and a calibrated robot once stalled roughly **0.54 m** short of Station A staging.
- Source analysis and a yaw-only replica suggested a plausible **sustaining** mechanism for an MPPI angular-command saturation lock, but did **not** establish the initial trigger or prove a general MPPI software defect.
- Rotation Shim was selected for evaluation, and independent bounded navigation supervision remained a future requirement.

**What changed after October 25:** Rotation Shim was actually integrated and tested; the development direction shifted toward structured warehouse routes; the interaction between route-path heading semantics and MPPI was tested directly; and a more suitable reverse policy and goal-yaw handoff were implemented and functionally validated. The original calibration and unresolved historical failures are **not retrospectively erased** by the newer successes.

### Evidence vocabulary

| Classification | Meaning in this document |
|---|---|
| **Observed / measured** | Values reported from Gazebo ground truth, topic logs, action results, configuration readback, or recorded trajectories. |
| **Source/config verified** | Findings Claude reported from inspection of the installed-version source code or loaded parameters. |
| **Supported interpretation** | A causal explanation consistent with comparisons and implementation details, without fully isolating all mechanisms. |
| **Unverified / open** | Not tested, insufficient evidence, or a plausible mechanism that was not isolated. |
| **Engineering decision** | A deliberately chosen architecture or operating policy; not a proof of empirical superiority. |

Measurements in this document are transcribed and reconciled from the reports listed in §13 and the engineering review decisions. The underlying raw simulation bags and every run log have **not** been independently replayed to create this continuation. Sample sizes are mostly **one or two runs per condition**.

---

## 2. Chronology and decision transitions

| Date / milestone | Work performed | What changed in our understanding | Decision |
|---|---|---|---|
| **26 Oct — Rotation Shim Phase 1** | Wrapped MPPI with `nav2_rotation_shim_controller`; 2 runs × 4 goals | All goals succeeded, but 1 Hz replanning caused unexpected midroute shim reactivation and larger final XY errors | **Do not adopt; pause** further shim work |
| **26 Oct — Structured routing** | Added a Route Server graph, custom route BT, optional mission integration | Route graph reliably constrained the ordinary forward trips; B → A departure nevertheless left its corridor by **1.71 m** | **Keep architecture; diagnose path/controller semantics** |
| **26 Oct — NavFn / MPPI profiles** | Compared original orientation-aware MPPI, geometric 2D-path settings, and forward-only motion | The improvement on routed B → A came from **MPPI semantics and reverse policy**, not the connector planner | Evaluate both planners under matched controller settings |
| **27 Oct — State Lattice follow-up** | Compared lattice reverse expansion on/off under 2D forward-only MPPI | Reverse-disabled lattice plans often stalled/dithered near Home; reverse-enabled lattice succeeded, despite path/controller direction mismatch | Reject the tested **lattice reverse-off** variant; do not select planner from single runs |
| **29 Oct — PreferForwardCritic** | Re-enabled reversing with a forward penalty; tested route, free-space, and useful back-out | Long reverse journeys dropped to **0.24–0.62 m** in ordinary trips, while **1.64 m** of useful back-out remained possible | Retain soft forward preference as leading candidate |
| **29 Oct — Consolidated validation** | Four focused tests, including rear obstacle and off-axis terminal yaw | Short reversing and geometric tracking remained functional; routed false-success yaw bug fixed | **Accept candidate as working simulation baseline** |
| **After validation** | Development planning | Controller experimentation had become less valuable than owner-authored routes and end-to-end missions | Next: **owner-designed graph**, station access, docking, mission execution |

---

## 3. Experiment 1 — Rotation Shim with State Lattice + MPPI (26 October)

### 3.1 Architecture and configuration

The first continuation experiment followed the October 25 decision by setting the Nav2 `FollowPath` controller to `nav2_rotation_shim_controller::RotationShimController`, with `nav2_mppi_controller::MPPIController` as its internal primary controller. The Smac State Lattice planner, MPPI critics and constraints, `StoppedGoalChecker`, and simulation wheel calibration were not intentionally changed.

Key explicit shim settings included:

```yaml
FollowPath:
  plugin: nav2_rotation_shim_controller::RotationShimController
  primary_controller: nav2_mppi_controller::MPPIController
  angular_dist_threshold: 0.785
  angular_disengage_threshold: 0.3925
  forward_sampling_distance: 0.5
  use_path_orientations: true
  rotate_to_heading_once: true
  rotate_to_goal_heading: true
  closed_loop: true
  rotate_to_heading_angular_vel: 0.8
  max_angular_accel: 1.0
  simulate_ahead_time: 1.0
```

The integration loaded and activated successfully, and loaded parameter values were checked. The work was preserved as **uncommitted/unpushed** on `feature/rotation-shim-integration`, separate from later routing and controller-profile branches.

### 3.2 Recorded results

Two test sequences each executed four goals starting at Station B (**8/8 `SUCCEEDED`**). These were functional checks, not reliability trials.

| Goal | Run 1 | Run 2 | Significant observation |
|---|---|---|---|
| B → A staging | 42.2 s; GT distance **0.212 m**, yaw −0.5° | 47.9 s; GT distance **0.181 m**, yaw −4.4° | Large initial rotation and midroute shim takeover |
| Short reverse 1 m | 5.2 s; GT distance 0.221 m | 4.3 s; GT distance 0.189 m | Reversal still worked; no gratuitous 180° initial spin |
| Forward 0.6 m + yaw 90° | 15.7 s; distance 0.212 m | 10.0 s; distance 0.189 m | Start/terminal rotation handoffs |
| In-place 180° | 4.8 s; yaw error ~0° | 5.1 s; yaw error ~0.2° | Direct terminal heading adjustment worked |

**Crucial negative finding:** The shim produced an initial **183–184°** in-place turn on B → A, followed by a further **59°** rotation at about **0.54 m** from the goal in one run and **174°** at about **0.91 m** in the other. Reported source inspection associated these activations with 1 Hz replanning and small changes to the lattice path's final pose; `rotate_to_heading_once` did not behave like a once-per-mission latch under changing path endpoints.

The B → A final XY errors (**0.18–0.21 m**) were worse than the earlier successful MPPI-only sample's median (~**0.05 m**). The shim took terminal control near the existing 0.25 m XY tolerance, allowing an accurate heading without requiring the robot to center further.

### 3.3 Conclusions and status

- **Observed:** Correct plugin integration, good final yaw on the tested cardinal orientations, and support for a simple reverse maneuver.
- **Observed:** Unwanted repeated alignment triggered during the route; initial and midroute rotations added behavior we did not want.
- **Open:** Collision safety while rotating, arbitrary requested yaw outside lattice orientation bins, narrow-aisle clearance, and any solution to the earlier calibrated stall.
- **Engineering decision:** **Pause Rotation Shim** and leave this implementation isolated. A proposed terminal-only follow-up was interrupted by a laptop restart and **was not accepted or validated**. Later successful navigation results must **not** be attributed to Rotation Shim: the accepted stack does not include it.

---

## 4. Experiment 2 — Structured warehouse routing (26 October)

### 4.1 Implementation, not yet an approved traffic design

The project gained a functioning Nav2 Route Server integration:

```text
Mission / GoToStation
  -> robot_executor -> NavigateToPose (optional custom route BT)
  -> Route Server computes directed graph route and dense path
  -> Smac planner only for bounded first/last-mile connections
  -> FollowPath (MPPI) tracks the route's dense path
  -> velocity smoother -> Collision Monitor -> diff_drive_controller
```

The custom tree is `src/amr_navigation/behavior_trees/navigate_on_warehouse_route.xml`. It is enabled with `structured_routing:=true`; disabling structured routing preserves the older free-space navigation behavior. The code also added optional `navigation_behavior_tree` to `robot_executor` without modifying the existing `GoToStation` or `ExecuteMission` action definitions.

The provisional graph, `src/amr_navigation/route_graph/warehouse_route_graph.geojson`, contains **6 nodes / 8 directed edges**. Its shared east–west lane is at **y = 2.705**; Home, `LANE_W`, `LANE_A_JCT`, and Station B are connected in both directions. A one-way spur leads from the junction to `A_APPROACH` then **A_STAGING (0.099, −0.175)**. It has **no outbound edge from A_STAGING**. The minimum reported clearance along graph edges was about **0.81 m** to a non-free map cell, versus a robot circumscribed radius of about **0.622 m**.

Station positions come from `amr_missions/config/stations.yaml`. The graph generator still hardcodes the docking staging X offset `−0.70 m`; hence station coordinates are only partly a single source of truth. The owner **did not approve the selected centerlines, one-way constraints, or station departure policy**. This graph is a **temporary test fixture** and must eventually be replaced by owner-authored operational routes.

Route Server supplies a dense path (0.05 m point spacing) to MPPI, rather than only loosely spaced waypoints to an unrestricted free-space planner. Smac's role is limited to short connectors. The original route BT rejected unbounded first/last-mile shortcuts. Later evaluation corrected the first-mile guard from **1.5 m to 2.5 m** because Route Server may prune the already-passed starting node; the final-mile guard remained **1.5 m**. The custom BT is adapted to the installed Jazzy 1.3.12 BT plugins, which did not ship a ready-made matching route tree in the tested installation.

### 4.2 Observed results

| Scenario | Outcome | Key measurements |
|---|---|---|
| Route computation, Home → B | Valid directed route | Nodes 1→2→3→6; **16.79 m** |
| Route computation, B → A staging | Valid directed route | Nodes 6→3→4→5; **17.69 m** |
| Home → B, routed | `SUCCEEDED` | Maximum route deviation **0.157 m**; final GT 0.041 m / 1.1° |
| Home → A staging, routed | `SUCCEEDED` | Maximum deviation **0.158 m**, including 90° turn; final GT 0.036 m / 0.5° |
| B → A staging, routed (2 runs) | Reached goal both times, but corridor departure failed | Maximum deviation **1.71 m**, reverse distance roughly **9.1–9.2 m** |
| Route edge 105 closed | Failure without robot motion | Error **405**, reopening restored route |
| Free-space behavior selected | `SUCCEEDED` for Station B | Optional feature preserved older navigation behavior |

Forbidden reverse edges were rejected at route-computation level. One forbidden-direction execution produced a misleading error **103** before additional BT error-code reporting was corrected; **that exact execution was not rerun** after the error-code change, though the blocked-edge case did return 405.

### 4.3 What the 1.71 m departure taught us

At Station B the robot faced **east (yaw 0)**, but the outbound graph edge pointed **west (yaw π)**. MPPI initially backed away while keeping the robot's nose east, then drifted south by as much as **1.71 m** from the intended centerline before recovering. A route path can geometrically constrain a global plan but **does not itself guarantee that the controller remains inside a physical lane**.

The first suspicion was a conflict between the travel-heading convention of Route Server's paths and MPPI's State-Lattice-oriented heading/reversing preferences. Subsequent experiments directly tested this suspicion.

**Unfinished routing issues:** no owner-approved graph or traffic rules; no outbound route from A staging; no blocked-lane wait/reroute policy; no docking handoff yet; static and dynamic obstacles not comprehensively tested; and the terminal path pose initially inherited the **last edge's yaw rather than the requested goal yaw** (later corrected in §8).

---

## 5. Experiment 3 — NavFn and MPPI path interpretation (26 October)

### 5.1 The critical distinction: “2D MPPI” is a profile, not another controller

The same Nav2 MPPI controller was used in both cases. “2D-path MPPI” denotes **three configuration changes** for paths without encoded feasible vehicle-heading/reverse maneuvers:

| Parameter | Earlier lattice-oriented MPPI | 2D-path interpretation | Engineering meaning |
|---|---|---|---|
| `PathAlignCritic.use_path_orientations` | `true` | **`false`** | Score geometric path alignment without treating every path pose's yaw as required vehicle heading |
| `PathAngleCritic.mode` | `2` | **`0`** | Favor facing the geometric direction of travel, not direction derived from encoded vehicle headings |
| `enforce_path_inversion` | `true` | **`false`** | Do not impose a lattice-style direction-cusp handoff on geometric route/NavFn paths |
| `vx_min` | `−0.35 m/s` | **unchanged** in the first 2D test | Negative velocity remains allowed; a forward preference is **not** a reverse prohibition |

A **separate** `2d_forward_only` profile adds `vx_min: 0.0`. These adjustments did **not** change MPPI's differential-drive motion model, 2.8 s prediction horizon, other critic weights, or simulated wheel calibration.

The reported Nav2 1.3.12 source inspection established that **NavFn's intermediate path poses carry yaw 0**, with the requested goal yaw at the end, while **Route Server path-pose yaw follows edge travel direction**. Neither is equivalent to a State Lattice vehicle-heading-annotated path with explicit reverse cusps. With `PathAlign.use_path_orientations: true`, NavFn's yaw-0 intermediate poses could incorrectly favor **east-facing** orientation on westbound travel. `PathAngle.mode: 2` could similarly interpret westward geometric motion as a reverse-oriented path.

`PathAngle.mode: 0` merely **penalizes** a backward-facing look-ahead. It does not set a hard rule against negative velocities; this was directly demonstrated.

### 5.2 Route-tracking comparison (Station B → A staging)

| Planner/profile | Max route deviation | GT reverse distance | Duration | Interpretation |
|---|---:|---:|---:|---|
| Lattice + original heading-aware MPPI | **1.71 m** | **9.1–9.2 m** | **80–81 s** | Prior problematic routed baseline; 2 runs |
| Lattice connector + 2D MPPI; reverse allowed | **0.41 m** | **17.98 m** | **65.4 s** | Geometry tracking improved but nearly whole journey reversed |
| NavFn connector + 2D MPPI; reverse allowed | **0.48 m** | **18.05 m** | **66.1 s** | Same qualitative backward-travel problem |
| NavFn + 2D MPPI; **forward-only** | **0.13 m** | **0 m** | **47.7 s** | Near-lane tracking and forward travel |

**Cause isolation:** The connector planner was **not invoked** in these B-start routed comparisons. The robot started on a route node and followed the Route Server path; the body of the trip bypassed both NavFn and State Lattice. Therefore, this comparison supports an effect of **MPPI path-interpretation and allowed velocity direction**, **not NavFn superiority**. Switching only the planner would not explain the route-departure improvement.

### 5.3 Free-space evidence (planner actually used)

With NavFn + 2D MPPI and reverse enabled, B → Home took **54.6 s** and drove approximately **16.55 m backwards**. Setting `vx_min: 0` produced an initial heading correction and forward travel, reaching Home in **44.6 s** with no reverse. Both reached the requested goal yaw in these runs. Separate yaw-90 free-space tests converged within **4.3°**.

NavFn's principal trade-off is that it generates cost-aware **2D geometric paths** rather than complete heading/reverse feasible trajectories, and its planning assumption is effectively circular rather than a full rectangular-footprint search. MPPI and the costmap/collision system still evaluate local feasibility; this was **not a demonstrated narrow-aisle guarantee**.

### 5.4 Finding and decision

**Finding:** The previous route-departure failure was strongly associated with inconsistent path semantics and a reverse cost policy—not proof that State Lattice was inherently unreliable. The pure 2D MPPI profile corrected the alignment interpretation but **did not prevent** very long backward trips. Forward-only did, but eliminated useful reverse motion as well.

**Decision:** Continue comparing compatible control policies. Preserve both planners; avoid pretending route-mode tests identified the better connector planner.

---

## 6. Experiment 4 — State Lattice + 2D forward-only MPPI (27 October)

A follow-up tested whether State Lattice could be retained while MPPI ignored path orientation and disallowed negative velocity. The lattice primitive file contained **80 forward-motion and 32 in-place rotation primitives** across **16 discrete headings**, with reverse travel arising through the planner's `allow_reverse_expansion` behavior.

### 6.1 Outcomes

| Test | Configuration | Reported outcome |
|---|---|---|
| Routed B → A staging | Lattice, reverse expansion ON; MPPI 2D forward-only | **49.5 s**, max route deviation **0.18 m**, **0 m** reverse; final GT 0.067 m / −0.3° |
| Free-space B → Home | Lattice reverse ON; MPPI 2D forward-only | **43.4 s**, final GT 0.13 m / −3.4°; **0 m** executed reverse |
| Free-space B → Home, run 1 | Lattice reverse OFF; MPPI 2D forward-only | **Cancelled** after 200 s wall / 123 s simulation; stopped ~**0.58 m** short and ~**89°** off yaw |
| Free-space B → Home, run 2 | Lattice reverse OFF; MPPI 2D forward-only | Succeeded but took **100 s**, with **462°** accumulated in-place rotation and wandering near Home |
| Free-space B → Home | Lattice reverse OFF; **original** MPPI | Succeeded in **112.9 s**, executed **1.5 m** reverse despite a forward-only lattice path |
| Free-space yaw-90 after problematic run | Lattice reverse OFF; MPPI 2D forward-only | Cancelled with **66.7°** yaw error |

The reverse-ON lattice could produce up to **2.05 m of planned reverse** and **2 cusps**, while forward-only MPPI still executed **zero** reverse. Thus `SUCCEEDED` did not mean it executed the planner's intended motion sequence.

When lattice reverse expansion was turned OFF, the resulting plans were valid and had no reverse or cusps, but contained many duplicate-position points representing in-place rotations. MPPI configured to ignore pose yaw could not directly interpret these points as required turning primitives. This is a **plausible** incompatibility; the precise stall mechanism was **not verified**. The original MPPI profile was also slow with the reverse-disabled plans, showing this was not simply caused by the 2D profile.

### 6.2 Conclusions

- **Reject as configured:** `allow_reverse_expansion: false` with the tested lattice setup produced unacceptable goal convergence in these few runs.
- **Provisional compatibility:** lattice reverse ON + 2D forward-only MPPI completed the selected scenarios, but its planner/controller reverse-cusp mismatch remains real and unvalidated in confined settings.
- **No planner superiority demonstrated:** NavFn forward-only achieved comparable free-space B → Home results (**44.6 s**, 0.082 m final XY error) with simpler geometric path semantics.
- **Decision:** NavFn + geometric-path MPPI remained the leading candidate; State Lattice profiles were retained for future scenarios that genuinely require motion-primitive/footprint-aware planning.

---

## 7. Experiment 5 — `PreferForwardCritic` with reversing restored (29 October)

The project owner clarified the intended policy: **forward travel should be normal, but reversing must remain possible when useful**. A global `vx_min: 0` prohibition was therefore undesirable, while unlimited negative-velocity sampling had yielded entire reverse journeys.

### 7.1 Configuration and verified mechanism

The test paired **NavFn + geometric-path MPPI** with a new overlay:

```yaml
# Conceptual contents of config/profiles/mppi_prefer_forward.yaml;
# loaded alongside mppi_2d_paths.yaml
controller_server:
  ros__parameters:
    FollowPath:
      vx_min: -0.35
      critics:
        - ConstraintCritic
        - CostCritic
        - GoalCritic
        - GoalAngleCritic
        - PathAlignCritic
        - PathFollowCritic
        - PathAngleCritic
        - PreferForwardCritic
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
```

The running controller parameters were read back and matched the intended overlay. Other critic weights, motion limits, footprint, horizon, and calibration were unchanged.

For a trajectory, the critic was reported to add (power = 1):

\[
J_{\mathrm{reverse}} = 5.0 \sum_k \max(-v_{x,k},0)\,\Delta t.
\]

At `vx_min = −0.35 m/s` and a **2.8 s** horizon, its largest per-trajectory addition is approximately **4.9 cost units** (assuming full-rate reverse across the horizon). It turns off within **0.5 m** of the path endpoint. The typical sizes of competing critic costs were **not measured**. Crucially, this cost prices **predicted backward distance**, not whether a reverse is logically necessary; it is a **soft preference**, not an allowed-motion or traffic-rule guarantee.

### 7.2 Main results

| Scenario | 2D MPPI, reverse allowed, no critic | 2D MPPI forward-only | 2D MPPI + PreferForward 5.0 |
|---|---|---|---|
| **Routed B → A staging** | **18.05 m** reverse; **0.48 m** max deviation; 66.1 s | **0 m** reverse; **0.13 m** max deviation; 47.7 s | **0.24 m** reverse; **0.15 m** max / 0.05 m mean deviation; **49.0 s** |
| **Free B → Home** | **16.55 m** reverse; **54.6 s** | **0 m** reverse; **44.6 s** | **0.62 m** reverse; **46.3 s** |
| **Synthetic pillar-pocket back-out** | Not run in this comparison | **0 m** reverse; **21.9 s**, 277° in-place | **1.64 m** reverse; **14.3 s**, 7° in-place |

All new prefer-forward test scenarios were run once. On the ordinary routes, the critic eliminated the **whole-route reverse preference** while preserving short departure reverse arcs. On a synthetic nose-near-pillar maneuver it permitted a substantially longer **1.64 m** back-out and was faster than the forward-only comparison.

### 7.3 What the useful-back-out test does *not* establish

The pillar experiment was **not a proof that reversing was necessary**. The warehouse floor is generally open enough for an in-place turn away from obstacles. The synthetic starting pose left the robot nose ~**2.6 cm** from a pillar; its forward-only control started about **6 cm farther away**, so the comparison was not geometrically identical. It demonstrated the **availability and potential efficiency** of reversing, not safety or automatic recognition of a required maneuver.

The existing LiDAR is 360° in plane, and the Collision Monitor uses predicted footprint motion; objects below the sensing plane are not thereby covered. Reversing with an object **behind** the robot was deferred to the consolidated test.

### 7.4 Conclusion

**Supported:** At weight 5.0, the critic removed the previously observed whole-route reversing while preserving both 0.24–0.62 m departure arcs and a useful 1.64 m synthetic back-out in the tested setups.

**Not supported:** Claims that the critic *knows* when reversing is necessary, guarantees a maximum reverse distance, will never reverse near a restricted station, or is safe for every rearward obstacle. These require mission-level direction rules, bounded maneuvers, and appropriate sensing.

**Decision:** Keep **one reverse-capable MPPI** with a forward preference as the leading simulation candidate rather than immediately implementing separate forward/maneuvering controllers.

---

## 8. Experiment 6 — Consolidated functional validation (29 October)

To reduce experiment overhead, we stopped running numerous isolated parameter sweeps. Claude performed **four primary functional scenarios in one validated simulator launch** with **no repeats**, using existing recording and analysis scripts. This is a **small integrated acceptance screen**, not statistical testing.

### 8.1 The terminal-yaw false-success defect and correction

**Previously observed defect:** A routed Station B goal at the usual goal position but requested **yaw 90°** reported `SUCCEEDED` while the actual robot heading remained near **0°** (roughly **−90.1°** error). The Route Server's dense path ended with the **last edge's direction** (0°). The route BT skipped the last-mile planner whenever the **position** difference between goal and route end was less than 0.1 m; the controller/goal checker then checked against the *path endpoint yaw*, not the user's requested yaw.

**Code correction:** The route Behavior Tree now **always computes a bounded last-mile connection** (with the route-end pose used as start) for a destination within **1.5 m** of the route endpoint. Even if the positions coincide, the NavFn connector supplies the **requested pose, including yaw**, as the final path pose. The route body and its graph-edge restrictions were not intentionally changed, and the existing goal tolerance of **0.25 m / 0.20 rad** was preserved.

**Measured regression:** With requested **+90°**, the last-mile plan ended at exactly 90°; the robot finished with **−3.5° yaw error** and **0.041 m XY error**. Thus this false-success example was corrected. A deliberately **unreachable yaw** scenario, to prove failure reporting, **was not tested**. Potential local connector shortcuts remain bounded geometrically, not necessarily forbidden by detailed warehouse policy; this must be reviewed with the owner-designed graph.

### 8.2 Four-test result table

All four tests returned Nav2 `SUCCEEDED` / code 0. Actual final pose was separately compared with the **requested** pose using Gazebo ground truth.

| Metric | T1: routed B → A staging | T2: free B → Home | T3: rear obstacle | T4: routed Home → B, yaw +90° |
|---|---:|---:|---:|---:|
| Simulation duration | **48.4 s** | **48.2 s** | **24.3 s** | **39.8 s** |
| Final XY error | **0.066 m** | **0.067 m** | **0.107 m** | **0.041 m** |
| Final yaw error | **−0.6°** | **−2.5°** | **−1.1°** | **−3.5°** |
| Forward travel | **17.34 m** | **16.05 m** | **3.61 m** | **16.91 m** |
| Reverse travel | **0.24 m** | **0.71 m** | **0.34 m** | **0.13 m** |
| Reverse segments | 1 | 1 | 1 | 1 |
| Max / mean route deviation | **0.14 / 0.05 m** | n/a | n/a | **0.16 / 0.05 m** |
| Minimum reported footprint clearance | **0.56 m** | **0.20 m** | **0.16 m while reversing; 0.05 m at goal** | **0.35 m** |

**T1: structured route and departure.** The robot turned toward the westbound route, reversed only **0.24 m**, kept its route deviation near the earlier prefer-forward result, and reached staging without an observed stall or loop. Initial heading correction was reported as **143° in 5.9 s** through the first 0.3 m of movement; this is not a measurement of stationary rotation alone.

**T2: free-space NavFn usage.** NavFn was actually invoked (reported **76 path messages**, totaling **661 poses**). The robot reversed once for **0.71 m** at B departure, then travelled forward and reached Home at the requested orientation. The 76 path messages reflect replanning during navigation, not 76 independent missions.

**T3: obstacle during reverse opportunity.** A box was placed behind the robot for the test. The LiDAR detected it at **1.508 m**, consistent with the reported geometric sum of the rear gap and LiDAR-to-rear distance. The local costmap marked its region lethal. The robot backed **0.34 m** of a roughly **0.66 m** rear gap while turning, then moved around the box on open floor. No box/footprint intersection was observed. It did **not** demonstrate an unavoidable-obstacle safe stop or low-obstacle sensing. The Collision Monitor briefly reported `APPROACH` near the goal, but 10 Hz command-output comparisons could **not** establish whether it intervened between samples.

**T4: off-axis requested yaw.** The robot completed the routed Home → B trip and aligned to the **requested +90° yaw**, despite the incoming lane pointing along +x (0°). The test reported **1.40 m** turning clearance against a **0.622 m** robot circumscribed radius. No false success in this tested case.

### 8.3 The important 5 cm clearance observation

In T3 the robot's footprint came within **0.05 m** of the box **at the goal** while rotating. The goal was deliberately placed only about **0.8 m** from the box face. This was mainly a **destination-placement and terminal-clearance problem**, not evidence that the controller drove through the box or that the reverse path was unsafe. Nevertheless, it demonstrates that the tested costmap/controller/goal tolerance combination can accept a very small terminal obstacle margin.

The reported clearance calculations used Gazebo ground-truth footprint geometry and **static map + inserted box**. They excluded the person model and any obstacle not represented by the map/box, especially low objects outside the LiDAR plane. **No minimum clearance policy was formally approved.** A prior engineering discussion suggested a provisional goal-placement allowance of approximately **0.4 m**, but it was **not validated or adopted as a universal minimum**; application-specific margins must be derived from footprint, localization uncertainty, sensing, braking/velocity, and site requirements. Precision docking necessarily has a different close-approach policy.

### 8.4 Acceptance decision

**Accepted as the working simulation development baseline:**

```text
Nav2 Route Server (provisional owner-unapproved graph)
  ├── Route Server dense path for normal warehouse corridors
  ├── NavFn for guarded short connectors / free-space navigation
  └── MPPI DiffDrive
        ├── 2D geometric-path interpretation
        ├── PreferForwardCritic: cost_weight 5.0
        └── reverse capability: vx_min -0.35 m/s
     -> velocity smoothing -> Collision Monitor
     -> calibrated diff_drive_controller -> Gazebo
```

This is **functional simulation-development acceptance only**. The route graph has **not** been approved. Docking, undocking, full missions, hardware safety, repeatability, and recovery behavior remain outside this acceptance scope. The old State Lattice, forward-only, and Rotation Shim experiments remain preserved for reference and possible targeted future work.

---

## 9. Decision register — what was accepted, rejected, or postponed

| Decision / feature | State as of 29 Oct 2026 | Basis and boundary |
|---|---|---|
| Simulator yaw correction `wheel_separation_multiplier: 0.92754` | **Retained** | Independent earlier system identification; applies to this Gazebo model only |
| NavFn free-space / connector planner | **Working development choice** | Simpler geometric semantics with the 2D controller profile; not proven superior to lattice in general |
| State Lattice planner | **Preserved, not primary** | May be valuable for footprint-aware, feasible maneuvers; route-mode trials cannot rank it against NavFn |
| Lattice `allow_reverse_expansion: false` variant | **Rejected as tested** | Repeated unacceptable near-Home dither/stall and long execution times |
| MPPI 2D-path interpretation | **Accepted with geometric paths** | Fixes path-yaw mismatch with NavFn and Route Server; unsuitable for deliberately orientation/cusp-encoded paths without further design |
| Hard forward-only MPPI (`vx_min: 0`) | **Preserved as comparator, not selected policy** | Solved long reversing but forbids useful back-outs |
| MPPI reverse enabled (`vx_min: −0.35`) + `PreferForwardCritic 5.0` | **Working simulation choice** | Short ordinary reverse, useful test back-out; soft preference, not a station traffic rule |
| Route Server architecture | **Implemented and retained** | Dense graph path genuinely influences controller route; not a hard lane-boundary guarantee |
| Current 6-node/8-edge route graph | **Provisional fixture only** | Claude-selected geometry/directions; owner wants to design and approve routes |
| Route final-yaw fix | **Implemented and tested** | Correct +90° requested pose achieved; unreachable-yaw failure path not tested |
| Rotation Shim | **Paused, not incorporated** | 8/8 functional successes but repeated unwanted heading corrections and poorer XY precision |
| Automated mission supervision | **Deferred, still needed** | No indefinite goal/spin/stall allowed in eventual unattended missions |
| Docking and undocking | **Not yet validated end-to-end** | Station A currently has docking code, but route-to-staging handoff and exit policy remain |
| Hardware/production readiness | **Not claimed** | No hardware, certified safety, repeated reliability, or full mission tests |

---

## 10. Unresolved engineering issues and evidence gaps

The following are **not arguments for another open-ended controller-tuning campaign**. They determine how we design routes, station operations, and later whole-system acceptance.

| Priority | Issue | Evidence / present status | Sensible trigger or next action |
|---|---|---|---|
| **High** | Owner-approved route geometry and traffic rules | Current lane centerlines, two-way edges and one-way A spur are generated from Claude's initial interpretation | Owner defines directed lanes, junctions, station approaches; Claude checks footprint clearance and implements |
| **High** | Station A route → staging → DockRobot integration | Existing executor still dispatches `DockRobot` with its own navigation to staging; route-first handoff not integrated | Implement and test dock staging handoff, then undocking/exit route |
| **High** | Reverse policy at stations | Soft critic still chose short unrequested 0.13–0.71 m departure reverses | Owner specifies station zones where reverse is allowed, restricted or must be an explicit maneuver |
| **High** | Terminal obstacle margins | T3 accepted **0.05 m** clearance near box, with collision monitor flicker | Define realistic station/goal placement and clearance policy; assess swept footprint and uncertainty |
| **High for unattended operation** | Mission navigation supervision | Historical terminal spin/stall; current success test suite has no mission-level watchdog | Implement bounded detection, cancellation, verified stopping, recovery/escalation before unattended operation |
| **Medium** | Startup lifecycle/goal-acknowledgment faults | Prior launches stuck on dropped `change_state` responses; occasional first goal unacknowledged | Diagnose startup infrastructure before unattended mission acceptance; don't confuse relaunch retries with root-cause fix |
| **Medium** | Route graph enforcement and obstacles | Route tracking is geometric; no hard lane polygon or full traffic-rule enforcement | Decide needed keepout/speed/lanes; test blocked/dynamic obstacle behavior on approved routes |
| **Medium** | First/last-mile connector constraints | 2.5 m first-mile guard; 1.5 m last-mile guard; path to requested orientation now correct | Ensure connectors cannot shortcut restricted approaches in final route design |
| **Medium** | Reverse obstacle detection and protection | Rear box visible to 360° planar LiDAR; low obstacles excluded | Test sensing coverage, rearward blind zones and collision monitor with real robot design |
| **Medium** | NavFn in narrow passages | NavFn's grid potential planning uses circular-clearance assumptions; rectangular full-footprint search not demonstrated | Validate only if owner-designed routes include tight corridors, doors or constrained turns |
| **Medium** | Unreachable terminal yaw failure | Requested yaw +90° now works; deliberately impossible yaw not tested | Include a negative acceptance case when constrained station poses are introduced |
| **Longer-term** | Repeatability and hardware transfer | Most results n=1–2; RTF often ~0.53–0.60; no hardware tests | Full-mission acceptance suite later; re-identify wheel kinematics on hardware |
| **Historical/open** | Earlier calibrated stall and saturation trigger | Previously observed; not reproduced in new profile's small test sample | Reopen only on recurrence with useful instrumentation or impact on actual mission behavior |

### Specific reporting cautions

- **Do not call 0.15 m deviation a lane guarantee.** It is robot-center distance to the *chosen graph polyline*. Actual footprint containment, dynamic obstacle clearance, localization uncertainty and traffic constraints are separate.
- **Do not call every `SUCCEEDED` correct by itself.** T4 showed why ground-truth **requested** XY/yaw comparison is required.
- **Do not call a 0.24 m reverse “necessary.”** The critic prefers shorter reversing but cannot infer its purpose; a short arc may simply be an optimizer trade-off.
- **Do not infer State Lattice inferiority from route-body runs.** The connector planner was not used in those runs.
- **Do not conflate startup failure and navigation failure.** Failed lifecycle activation or unacknowledged goals were excluded from valid controller outcomes, but remain infrastructure faults.

---

## 11. Engineering lessons and methodology changes

### 11.1 Match path semantics to the controller

The decisive conceptual issue was whether path-pose yaw meant **the robot's actual desired heading** (State Lattice feasible motion, possibly reversing), **the geometric direction of the route** (Route Server), or merely a placeholder (NavFn intermediate points). With original MPPI settings, these meanings were conflated. Correct semantic interpretation mattered more than swapping the planner's name.

### 11.2 A soft motion preference is not a mission rule

`PreferForwardCritic` solved the observed whole-route reversing at the tested weight, but does not know whether the warehouse permits reversing at a station, in an intersection, or near a person. The route graph and mission/station maneuver policy must encode **where reversing is allowed**, while safety sensing must govern whether a commanded maneuver is collision-free.

### 11.3 Separate measured behavior from causal explanation

The 1.71 m lane excursion, repeated 16–18 m backwards travel, 0.24 m reverse departure, and corrected +90° goal are measurements. The exact optimizer reason for preferring any one arc is less certain without full critic-cost and rollout evidence. In particular, the previous controller-saturation investigation remains a **limited, historical explanation**, not a universal defect claim.

### 11.4 Test the actual architecture path

With Route Server providing the primary path, NavFn and State Lattice may be **absent from the controller loop for most of a mission**. Planner comparison requires an experiment where the planner actually generates a path, otherwise it measures controller behavior only. Claude eventually verified invocation using path publications/connector recordings.

### 11.5 Verify requested goals rather than internal representations

The route BT's path endpoint silently replaced the user's goal yaw. A clean action success response was therefore false at the mission level. The correction established that terminal pose semantics are an **interface contract**, not simply a controller tuning parameter.

### 11.6 Focused integrated tests are more useful during feature development

We moved from many limited parameter trials toward **one four-scenario integrated validation** that simultaneously screened reverse policy, free-space planning, rearward obstacles, footprint margins, route tracking, and off-axis goal yaw. This is an efficient basis for development acceptance; later full-mission testing must still assess repeatability and failure behavior. The user explicitly prefers **bounded, decision-oriented experiments** and no repeated large campaigns without a specific unresolved requirement.

### 11.7 Preserve and isolate work, but keep the repository workflow simple

The previous State Lattice baseline, Route Server milestone, NavFn/MPPI comparisons and Rotation Shim trial were preserved separately. The project owner prefers working in the **existing `warehouse_amr` directory**, not proliferating separate project directories/worktrees. All completed, validated milestones should be **committed and pushed** to GitHub feature branches; uncommitted or local-only work must be explicitly reported and protected.

---

## 12. Current implementation, reproducibility, and next milestone

### 12.1 Working simulation configuration

The accepted configuration is a **selectable profile**, not necessarily the default launch values in every source tree. Reproduction command from the final validation report:

```bash
source /opt/ros/jazzy/setup.bash
# From an appropriately built and sourced warehouse_amr workspace:
source install/setup.bash
ros2 launch amr_bringup simulation_navigation.launch.py \
  planner:=navfn \
  mppi_path_interpretation:=2d_prefer_forward \
  structured_routing:=true
```

The relevant overlay files include:

```text
src/amr_navigation/config/profiles/planner_navfn.yaml
src/amr_navigation/config/profiles/mppi_2d_paths.yaml
src/amr_navigation/config/profiles/mppi_prefer_forward.yaml
src/amr_navigation/behavior_trees/navigate_on_warehouse_route.xml
src/amr_navigation/route_graph/warehouse_route_graph.geojson
src/amr_navigation/route_graph/tools/make_route_graph.py
src/amr_missions/config/stations.yaml
src/amr_missions/amr_missions/robot_executor.py
```

**Note:** The profile overlays were applied at launch and verified by parameter readback. They were not automatically made the default for every launch in the experimental work. An accepted **development baseline** means an intentional engineering choice, not an assertion that `main` or the default launch already selects these values.

### 12.2 Repository and commit provenance

Recorded GitHub branches and key commits:

| Branch | Key recorded commit(s) | Significance |
|---|---|---|
| `feature/bidirectional-navigation-baseline` | `88a7e70` | Pushed, preserved calibrated earlier navigation state |
| `feature/rotation-shim-integration` | Trial files still uncommitted/unpushed in reported working state | Separate postponed shim work |
| `feature/structured-warehouse-routing` | `d22907f`, `3f62022`, `67ddad2`, **`9d1d8d7`** | Route graph, BT, optional mission integration, report and figures |
| `experiment/navfn-mppi-evaluation` | `ce238c7`, `c703fda`, `7f21069`, `3299636`, **`e067834`** | NavFn profiles; lattice comparison; PreferForwardCritic; consolidated yaw fix/validation |

The experiment branch at **`e067834`** was reported and subsequently verified on GitHub as containing the latest consolidated navigation work. As of the repository inspection for this continuation, **a separate `feature/navigation-development-baseline` branch was not present among the listed remote branches**. A later Claude task to create that named baseline should be verified on completion rather than assumed to have occurred. `main` was reported untouched throughout these experiments.

Useful detailed source files:

```text
docs/STRUCTURED_ROUTING.md
docs/NAVFN_MPPI_EVALUATION.md
docs/LATTICE_2D_FORWARD_EVALUATION.md
docs/PREFER_FORWARD_CRITIC_EVALUATION.md
docs/CONSOLIDATED_VALIDATION.md
docs/figures/val_T1_routed_B_to_Astaging.png
docs/figures/val_T2_free_B_to_Home.png
docs/figures/val_T3_rear_obstacle.png
docs/figures/val_T4_routed_Home_to_B_yaw90.png
investigations/navfn_mppi_evaluation_2026-10-26/
investigations/consolidated_validation_2026-10-29/
```

The reports also reference local-only raw GT CSVs, temporary logs and earlier Rotation Shim recordings. Their absence from Git is intentional; the published metrics/figures are **not substitutes for preserving the original raw data** where deeper post-hoc checks might be necessary.

### 12.3 Development plan, after the documented experiments

**Immediate next milestone: owner-designed warehouse routing.** The owner—not Claude—defines corridor centerlines, directionality, junctions, station entry/exit, turning areas and reverse permissions. Claude should prepare a visual editor over the actual occupancy map, show station poses and footprint clearance, save/reload the owner-authored GeoJSON, validate connectivity and geometry, and integrate the approved graph without silently regenerating an alternative traffic layout. The provisional graph remains a test fixture until that review.

**Following milestones:** Integrate structured route arrival at Station A staging with `DockRobot`; then define a controlled `UndockRobot`/exit behavior, establish station-specific reverse rules and operational clearance, and connect these to complete `ExecuteMission` workflows. Before unattended operations, add bounded health supervision for prolonged spins, stalls, missing feedback, cancellation and safe-stopping verification. Reserve larger repeatability studies for **complete mission acceptance**, rather than rerunning an entire navigation experiment suite at every subsystem change.

---

## 13. Source record and traceability

### Preceding documents supplied for context

1. `MTS_AMR_Navigation_Investigation_Lessons_and_Rotation_Shim_Decision_2026-10-25(1).md` — Earlier investigation and decision boundary; includes wheel-calibration data, saturation study and unresolved stall.
2. `Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23(1).md` — Earlier run-level evidence for terminal/far-field spins and calibrated stall.

### Follow-up documents used to construct this continuation

3. `Rotation_Shim_Integration_Phase1.md` — 26 October Phase 1 configuration, 8 goals, replanning-triggered shim behavior, and unresolved risks.
4. `docs/STRUCTURED_ROUTING.md` — Route graph, Route Server BT architecture, geometry, station access, edge restrictions and routed test data.
5. `docs/NAVFN_MPPI_EVALUATION.md` — Source-based explanation of NavFn/Route Server yaw semantics; 2D-path MPPI profiles and free/routed comparisons.
6. `docs/LATTICE_2D_FORWARD_EVALUATION.md` — Reverse expansion ON/OFF tests, primitive-set behavior and terminal convergence evidence.
7. `docs/PREFER_FORWARD_CRITIC_EVALUATION.md` — `PreferForwardCritic` code interpretation, three scenario results, synthetic pillar back-out and known confounds.
8. `docs/CONSOLIDATED_VALIDATION.md` — Four final acceptance tests, obstacle and clearance evidence, correction of the route final-yaw false success, and simulation acceptance verdict.
9. Commit histories on `feature/structured-warehouse-routing` and `experiment/navfn-mppi-evaluation` — Implementation and documented changes, including `e067834`.

For reproducibility, use the **branch/commit-matched version** of each report and configuration. The figures are cited by repository path and are **not embedded** in this Markdown artifact; their filenames should be preserved if this record is placed alongside them in the repository. Historical source reports may reflect their **own original status dates**, which this continuation supersedes only for the post-October-25 decisions—not for the raw measurements they recorded.

---

## 14. Bottom-line engineering conclusions

1. **Rotation Shim was actually evaluated, not merely proposed.** It produced good final yaw and 8/8 goal successes, but repeated rotations from replanning and degraded terminal XY precision meant it was not adopted. The unfinished work stays preserved.
2. **Structured route navigation worked, but the initial 1.71 m departure excursion showed that a route graph alone does not enforce lane behavior.** Path orientation semantics and controller direction choices are part of route execution correctness.
3. **The routed MPPI problem was primarily a controller/path-interpretation and reverse-policy issue, not evidence that NavFn was a superior planner.** The tested route-body runs bypassed both connector planners.
4. **“2D MPPI” is a configuration of the original MPPI**, not a new algorithm. It stops using geometric path poses as if they carried intentional vehicle heading and reverse cusps.
5. **Forward-only MPPI solved whole-route reversing but removed a useful ability.** Adding `PreferForwardCritic` while retaining `vx_min: −0.35` provided a better compromise in the tested scenarios: short reverses during normal travel and a feasible longer back-out when beneficial.
6. **Planner/controller consistency remains important.** Lattice reverse-disabled plans had poor terminal behavior in this setup, while reverse-enabled lattice could succeed with a forward-only controller by ignoring planned reverse segments; neither supports a general conclusion about planner superiority or confined-space behavior.
7. **The route-to-goal interface contained a real false-success defect.** The custom BT's last-mile correction restored the requested yaw and succeeded on an intentionally off-axis +90° terminal goal.
8. **Four integrated functional checks were sufficient for a development choice, not a reliability claim.** The accepted working simulation profile is NavFn + 2D MPPI + PreferForwardCritic 5.0 + reverse enabled + Route Server, with calibrated simulation wheels and existing goal checks.
9. **Important limitations are still real:** 5 cm observed terminal clearance in the rear-box test, incomplete sensor coverage, no docking/undocking integration, imperfect startup readiness, no owner-approved graph, and no independent mission supervisor.
10. **The next engineering value is in the system, not more MPPI parameter experiments.** Let the owner design the warehouse routes, then integrate station maneuvers, docking and full missions; reopen controller internals only if a concrete new failure warrants it.

**Closing status, 29 October 2026:** Navigation architecture **accepted for simulation development**; route layout **unapproved**; Rotation Shim **paused**; remaining system and safety work **explicitly open**.
