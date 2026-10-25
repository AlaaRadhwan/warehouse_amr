# MTS_AMR — Navigation Investigation, Lessons Learned, and Rotation Shim Decision

**Project:** `warehouse_amr` / MTS_AMR  
**Status date:** 25 October 2026  
**Investigation covered:** September–24 October 2026, with major measured work on 20–24 October  
**Stack investigated:** ROS 2 Jazzy, Nav2 Jazzy MPPI 1.3.12, Smac State Lattice, Gazebo, differential-drive control  
**Decision recorded:** Evaluate **Rotation Shim wrapping the existing MPPI controller**, retaining Smac State Lattice and the calibrated wheel model initially. **Rotation Shim has not yet been implemented or validated.**  
**Document type:** Consolidated engineering record and decision log; not a publication claiming a proven root cause or statistical reliability.

> **Executive status.** The AMR's navigation problems were real and not limited to an incorrect goal tolerance or a single bad path. The strongest verified physical-model result is a repeatable **1.07812 actual/commanded yaw-rate ratio** in the original Gazebo wheel configuration, traced empirically to an effective wheel separation smaller than the nominal wheel-center spacing. A launch-time correction to `wheel_separation_multiplier: 0.92754` removed that steady rotational mismatch. Navigation then improved, but later tests showed that **uncalibrated navigation can also succeed** and a **calibrated run can stall**. A later instrumented failure, `e01`, showed MPPI continuously commanding +1.0 rad/s while its feedback measured about +1.079 rad/s. Source inspection and a simplified replica provide a plausible **sustaining** mechanism for this saturation, but do not identify the initial trigger or prove the full-controller mechanism. We therefore decided **not to ignore the risk**: we will test explicit terminal heading handling with Rotation Shim, retain bounded navigation failure supervision as a required safeguard, and validate the resulting system before calling it dependable.

---

## 1. Purpose, scope, and evidence standard

This document consolidates **what happened, what was measured, what was inferred, what was corrected, what remains open, and what we decided to do next**. It is intended for future project development, technical writing, and handoff to AI coding collaborators.

It draws on the project investigation reports listed in [§16](#16-source-records-and-artifacts), including the 22 October consolidated report, the 23 October variability report, the 23 October instrumented root-cause report, the 24 October source-check addendum, the recorded test/configuration tables, and subsequent design discussions.

Evidence terms used throughout:

| Label | Meaning |
|---|---|
| **Measured** | Direct observation in reported Gazebo ground truth, rosbag, odometry, action result, run log, or controlled identification test. |
| **Source/config verified** | Reported inspection of configuration or the source corresponding to the installed MPPI version; does not imply the full runtime was replayed. |
| **Supported explanation** | Consistent with several observations and/or code behavior, but not established by a discriminating experiment on the full stack. |
| **Hypothesis/open** | Plausible but unverified, or contradicted by some observations. |
| **Engineering decision** | Chosen design direction, not a measured improvement. |

**Important boundary:** These documents include both full logs and compact reports at their **original repository locations**. The earlier uploaded evidence ZIP is a compact report-and-figure package, **not a complete archive of all rosbags**. This consolidation evaluates reported measurements and methodological consistency; it is not a fresh reprocessing of every raw bag.

## 2. System context and operational objective

The robot is a simulated warehouse differential-drive AMR (nominally about **1.0 × 0.6 m** body; planning footprint approximately **1.0 × 0.74 m**). Its current navigation chain is conceptually:

```text
Mission executor / NavigateToPose
                |
                v
       Smac State Lattice
       (global path, reversing permitted)
                |
                v
          MPPI DiffDrive
       (local trajectory optimization)
                |
                v
         Velocity smoother
                |
                v
         Collision Monitor
                |
                v
       diff_drive_controller
                |
                v
              Gazebo
```

The robot also has a staging-to-precision-docking architecture: ordinary Nav2 navigation approaches a station's staging pose; an AprilTag-based precision docking phase is distinct. **The failures documented here occurred during ordinary/staging navigation, not because of AprilTag docking.**

Key experimental configuration (verify loaded parameters again before new tests):

| Element | Relevant setting during late investigation |
|---|---|
| ROS / Nav2 | ROS 2 Jazzy / reported `nav2_mppi_controller` 1.3.12 |
| Global planner | Smac State Lattice, reverse expansion enabled |
| MPPI model | `DiffDrive`; `vx_min = -0.35 m/s`, `vx_max = 0.8 m/s` |
| MPPI angular constraints | `wz_max = 1.0 rad/s`, `az_max = 1.0 rad/s²`, `model_dt = 0.05 s` |
| MPPI direction changes | `enforce_path_inversion: true`; inversion tolerances ~0.15 m and 0.30 rad |
| MPPI plan window | `prune_distance = 2.5 m` |
| Goal checker | `StoppedGoalChecker`: XY 0.25 m, heading 0.20 rad (~11.46°), stopped-speed checks |
| Driven wheels | Nominal radius 0.100 m, nominal center separation 0.690 m; width 0.050 m |
| Kinematic correction | `wheel_separation_multiplier`: original **1.0**; calibrated **0.92754** |
| Controlled B→A destination | Station A staging `(0.099, -0.175, yaw 0)` in `map` |
| Station B | `(8.716, 2.705, yaw 0)` |

The engineering requirement is not merely that the robot sometimes reaches the goal. It must **converge within the configured pose and stopped-velocity criteria**, and when navigation cannot converge, it must provide a **bounded, observable, safe failure outcome**, not spin indefinitely or leave a goal active without useful progress.

## 3. Failure modes: separate phenomena, not one interchangeable bug

### 3.1 Raw State Lattice terminal loops

Earlier tests produced approximately **3.0–3.3 m loops near a goal** despite the robot already being close to the target position. Inspecting `/unsmoothed_plan` showed that some loops existed **in the planner output before smoothing**. The smoother was therefore not the origin of those raw-loop geometries.

Changes to analytic expansion length and rotation penalty did not eliminate the issue in the isolated tests. The planner supported in-place-rotation primitives, so the loops were **not simply forced by a lack of rotation primitives**. Why the planner sometimes preferred a loop remained unresolved.

### 3.2 Path association and transformed-plan behavior

An earlier forward-only/yaw-π investigation documented a global goal-coincident loop that MPPI's transformed local plan sometimes represented as a very short segment near an overlapping endpoint. One example reduced a ~3.23 m path to ~0.14 m. This demonstrated a **real path-association/representation concern** for that earlier scenario.

**Do not generalize this explanation to the later `e01` spin.** The October 23 instrumented analysis found that, for `e01`, transformed-plan lengths largely matched what the actual MPPI path-handler source predicted. In other words: the older loop-association observation was genuine, but it was **not shown to be the sustaining mechanism in `e01`**.

### 3.3 Indefinite angular rotation

The central severe symptom was a robot which arrived near a goal or was still several metres away, then rotated continuously while MPPI's angular command saturated at roughly **±1.0 rad/s**. The goal checker correctly refused to declare success while the robot continued moving. Two distinct locations were observed:

- **Terminal spin:** near the final target pose (earlier bidirectional bag, uncalibrated `B1`, and instrumented `e01`).
- **Far-field spin:** several metres before the goal (`H1` and an October 22 uncalibrated control).

They share the saturation signature but their **triggering geometry and full causal chains were not proven identical**.

### 3.4 Calibrated terminal stall

A separate run (`B2`, 23 October, multiplier **0.92754**) stopped around **0.54 m short of Station A**, with the goal about **106° from its current heading**, for approximately **45 simulation seconds**. Gazebo ground truth showed negligible late angular movement. Nav2 reported a `Failed to make progress` event, cleared the local costmap and retried, without restoring useful motion.

This was **not** an MPPI pinned-yaw spin and **has not been explained or reproduced under full instrumentation**. Rotation Shim's final-heading behavior, which is expected to engage around XY goal tolerance, **must not be assumed to fix a stall outside that tolerance**.

### 3.5 Startup/infrastructure failure

In the October 23 instrumented batch, **4 of 14 launches failed before any navigation goal** because of dropped lifecycle/middleware responses or a Gazebo service that did not complete. Those launches were excluded from navigation statistics, correctly, but they remain a separate system-readiness concern. A robot that cannot reliably bring up its software is not operationally dependable even if its controller works after startup.

## 4. Chronological engineering investigation

| Stage | Test or intervention | Result / lesson |
|---|---|---|
| Early work | Smac2D + RPP and State Lattice + RPP explored | Navigation problems were present before the final MPPI setup; don't attribute every early failure exclusively to MPPI. |
| Planner isolation | Compared `/unsmoothed_plan` and `/plan` | Some terminal loops were generated by State Lattice, not the smoother. |
| Planner changes | Analytic expansion 3.0→2.0 m; rotation penalty 5→2 | Neither adjustment alone removed the original terminal behavior. |
| Command pipeline | Corrected stale smoothed `TwistStamped` timestamps | Resolved a genuine timing/stamping issue, but later persistent angular commands originated upstream in MPPI. |
| MPPI approach | Reduced `PathFollowCritic.threshold_to_consider` 1.4→0.10 m | Improved approaches, did not guarantee final yaw convergence. |
| Yaw-π test | Aligned final station heading with travel | Improved approach geometry, but nearby loops/failure still occurred. Target yaw alone was not the entire cause. |
| Bidirectional navigation | Enabled reverse planning, negative MPPI `vx`, cusp/inversion handling and stopped goal checking | A straight reverse test succeeded; full B→A still spun. Reverse support alone did not solve the failure. |
| Spin diagnosis | Compared `/cmd_vel_nav`, wheel odom, IMU, fused `/odom`, MPPI prediction | Discovered a sustained angular velocity discrepancy and saturation. |
| Independent identification | Disabled Nav2 and commanded controlled linear/angular motion | Established a repeatable angular scale error, not a forward-speed scale error. |
| Calibration | Set effective wheel separation with multiplier 0.92754 at launch | Removed steady yaw-rate mismatch; 3 initial calibrated B→A runs succeeded. |
| 23 Oct variable reruns | 7 partially instrumented runs using Home or B starts | Both spin and success occurred with uncalibrated plant; calibrated stall showed the problem was not comprehensively solved. |
| 23 Oct instrumented study | 10 valid B-start runs, balanced arms | Captured one high-quality terminal spin (`e01`), nine successes, no reproduced stall; narrowed where sustained spin appears. |
| 24 Oct source + offline comparison | Compared `e01` against successful `p03` and `e05`; checked MPPI 1.3.12 source and reduced yaw model | Identified plausible mechanism sustaining command saturation but not explaining why the initial command became larger. |
| 25 Oct design decision | Reconsidered NavFn / Rotation Shim versus continuing with current architecture | Chose an **isolated Rotation Shim evaluation first**, together with future failure supervision and functional validation. No implementation yet. |

## 5. The most conclusive technical result: yaw-kinematic identification

### 5.1 What was actually measured

Independent commands with Nav2 inactive gave the following steady yaw rates **before calibration**:

| Commanded yaw (rad/s) | Gazebo ground truth (rad/s) | Raw wheel odom (rad/s) |
|---:|---:|---:|
| +0.40 | +0.4312 | +0.4000 |
| -0.40 | -0.4312 | -0.4000 |
| +0.70 | +0.7547 | +0.7000 |
| -0.70 | -0.7547 | -0.7000 |
| +1.00 | +1.0781 | +1.0000 |
| -1.00 | -1.0781 | -1.0000 |

Mean actual/command ratio: **1.07812**. IMU angular rate tracked Gazebo ground truth; raw wheel odometry tracked the nominal kinematic calculation instead. Fused `/odom` angular twist was also close to the true/IMU rate in the spinning runs. Thus the Nav2 controller was **being told about the real, faster rotation**.

By contrast, straight tests at ±0.30 and ±0.60 m/s showed approximately unity forward-speed gain. The longitudinal model did not exhibit a comparable scaling error.

### 5.2 Physical interpretation

For differential drive:

\[
  v = \frac{r}{2}(\dot\phi_R+\dot\phi_L), \qquad
  \omega = \frac{r}{b}(\dot\phi_R-\dot\phi_L).
\]

Using nominal driven wheel radius **0.100 m**, wheel-center separation **0.690 m**, and measured yaw gain **1.07812** gives an empirical effective separation

\[
  b_{\mathrm{effective}} \approx \frac{0.690}{1.07812} \approx 0.640\ \mathrm{m}.
\]

The URDF and observed wheel-joint locations still had a center spacing of ~0.690 m. Controlled changes to wheel collision-cylinder width changed the identified effective separation (reported examples: width 0.10→0.590 m; 0.05→0.640 m; 0.02→0.6723 m). This supports a **simulation wheel-ground contact geometry effect**. The actual contact points were not directly measured, so the contact explanation remains an empirical interpretation rather than a completely established microscopic model.

### 5.3 Intervention and its limits

The correction was:

```yaml
# src/amr_control/config/controllers.yaml
wheel_separation: 0.69
wheel_radius: 0.10
wheel_separation_multiplier: 0.92754
```

The effective controller value becomes ~0.640 m. It was important to **load the configuration at controller startup**; one attempted runtime parameter update did not actually alter the controller's operative wheel behavior, and that invalid control run was excluded.

After calibration, the report found angular-response and yaw-estimation agreement was much better, and the initial B→A validation returned **3 successes in 3 calibrated trials**. A correctly launched uncalibrated control spun far from the goal.

**Defensible conclusion:** the nominal Gazebo wheel configuration contained a reproducible yaw-response mismatch, and correcting it was justified independently of Nav2 outcomes. Its removal improved the tested navigation behavior. **Not defensible:** “calibration guarantees navigation success,” “the physical AMR wheel separation is 0.640 m,” or “every spin and stall shared that one cause.” Real hardware requires its **own identification**.

## 6. 23 October variability: the reason the early success claim was revised

The first October 23 re-runs deliberately switched between multiplier `1.0` and `0.92754`, sometimes launching from **Home → B → A**, and sometimes **directly at B**. These are different initial conditions and are **not interchangeable statistical trials**.

| Run | Initial condition | Multiplier | Reported result |
|---|---|---:|---|
| H1 | Home → B → A | 1.0 | Far-field spin; MPPI commanded near +1.0 rad/s; never closer than ~5.44 m |
| H2 | Home → B → A | 0.92754 | Success |
| H3 | Home → B → A | 1.0 | Success |
| H4 | Home → B → A | 0.92754 | Did not finish within wall cap; insufficient pose/command evidence |
| B1 | Spawn at Station B | 1.0 | Terminal spin; ~+1.079 rad/s true yaw; ~3,356° accumulated rotation |
| B2 | Spawn at Station B | 0.92754 | Stall ~0.54 m short of goal; no sustained spin |
| B3 | Spawn at Station B | 1.0 | Success; arrived reversing; final ground-truth XY error ~0.035 m |

This established that **the uncalibrated configuration did not fail deterministically**. It could finish B→A, and it could fail either far from the goal or at the goal. It also showed that calibration did not establish reliability, because a non-spin failure was observed with it.

**Recording caveat:** Several runs had no rosbag, and failed runs killed by a wall-clock cap could lack an action-result JSON. The later instrumented study was designed to avoid repeating these evidence gaps.

## 7. The instrumented B-start comparison: ten valid navigation trials

The next study held the spawn at Station B and interleaved the two controller configurations. It recorded global/transformed plans, MPPI optimal trajectory, the full velocity command chain, wheel/fused odometry, IMU, AMCL, TF, Gazebo ground truth, Nav2 action status, costmap and system-monitor data. Outcome classification was documented **before** evaluating the pilot:

- **SUCCESS:** Nav2 returned `SUCCEEDED`, with an additional check against ground-truth XY/yaw tolerance.
- **SPIN:** mean signed true yaw rate magnitude ≥0.8 rad/s over a 15-s window while mean speed ≤0.25 m/s, and no action success.
- **STALL:** ≤0.25 m net displacement and low yaw rate over 20 s, while goal remained unmet and active.
- Other failures and startup-inconclusive launches were recorded separately.

### 7.1 Outcomes

| Run | Multiplier | Outcome | Time after goal, sim s | Peak MPPI \|ω\|, rad/s |
|---|---:|---|---:|---:|
| p01 | 1.0 | Success | 47.4 | 0.69 |
| p03 | 1.0 | Success | 42.8 | 0.55 |
| e03 retry | 1.0 | Success | 39.7 | 0.59 |
| e05 | 1.0 | Success | 40.6 | 0.60 |
| **e01** | **1.0** | **Terminal spin** | **51.3, cut after detection** | **1.00** |
| p02 | 0.92754 | Success | 50.7 | 0.46 |
| p04 retry | 0.92754 | Success | 57.1 | 0.40 |
| p06 | 0.92754 | Success | 57.2 | 0.39 |
| e02 | 0.92754 | Success | 57.4 | 0.37 |
| e04 | 0.92754 | Success | 45.1 | 0.39 |

**Totals within this particular fixed-start study:** `1.0`: **4/5 successes, 1/5 terminal spin**; `0.92754`: **5/5 successes**. **No calibrated stall reproduced.** Startup failures were **4/14 launches** and excluded from this navigation denominator. Trial stopping was time-limited, not an exhaustive reliability campaign.

No reliable failure probability can be claimed from these counts. The report gives an exact two-sided 95% interval of approximately **0.5%–72%** for 1/5 spin in the uncalibrated arm, and a two-sided 95% upper limit near **52%** for 0/5 failures in the calibrated arm.

### 7.2 Two approach families

The recorded paths separated into:

1. **Forward with large initial terminal heading error** (~120–145° on first entering 1 m), followed by a substantial turn and typically a forward/reverse transition. This occurred in four uncalibrated trials and one calibrated trial; `e01` was the one failure.
2. **Reverse approach toward the goal**, with smaller final heading corrections and lower peak angular commands. These five trials all succeeded.

This is an **association in a small sample**, not evidence that reverse arrival is guaranteed safe or that one approach family caused the failure. The two arms also chose different approach families with different frequencies, so configuration effect and route choice were **confounded**.

## 8. The well-recorded `e01` spin

### 8.1 Observed sequence

Run `e01` (uncalibrated, directly spawned at Station B) behaved normally during cruise. Approximately 27 seconds after the goal, it entered the final metre **moving forward**, with about **+145° heading error** and an MPPI yaw command around **+0.49 rad/s**. Its angular command continued to rise. The robot crossed its target heading while still about 0.64 m away—**outside** the 0.5 m region in which `GoalAngleCritic` became active in the checked configuration.

It entered the final 0.5 m with about **−18° heading error**, yet still had a **+0.58 rad/s** angular command pointing away from the shorter heading correction. Later it reached the goal position, rotated in place, and saturated its command at **+1.0 rad/s**. Its measured yaw rate was about **+1.079 rad/s**. It crossed the goal heading repeatedly without deceleration or Nav2 success. The online watchdog recorded the event and cancelled the run after collecting more data; the resulting `CANCELED` status was **experiment-driven**, not ordinary navigation success.

### 8.2 Command, executed motion, and estimator

In the stable spin window:

| Signal | Approximate yaw rate |
|---|---:|
| MPPI `/cmd_vel_nav` | +1.001 rad/s |
| Final commanded `/cmd_vel` | Same as MPPI within measurement resolution |
| Raw wheel odometry | +1.001 rad/s |
| Gazebo true body yaw | +1.079 rad/s |
| IMU yaw | +1.079 rad/s |
| Fused `/odom` yaw-rate feedback | +1.079 rad/s |

The downstream command chain did not introduce a separate 1.0 rad/s spinning command: **MPPI itself was already producing it**. The actual plant was faster than the command, and Nav2's fused odometry reflected the faster rate. In the 10 instrumented trials, the controller's wall-time loop cadence was approximately 20 Hz with **no observed missed cycles** in the failure; localization errors were comparable to the successful runs. These measurements weaken a simple timing- or AMCL-error explanation **for `e01`**, not necessarily for every past failure.

### 8.3 What the global and local plans showed

The plan supplied to MPPI still reached the relevant endpoint, and transformed-path length usually matched the source-defined path handling. During the spin, frequent replanning produced short, changing and sometimes single-pose plans. They might be **consequences of the rotating start pose**, not necessarily the initial cause of the sustained rotation.

The MPPI **optimal predicted trajectory itself continued to rotate through the desired heading**, even at moments where a shorter braking movement appeared preferable. That localizes the observed bad choice to MPPI's trajectory selection/output, but does not alone identify its internal cause.

## 9. 24 October offline addendum: source mechanics and the earliest divergence

The last analysis deliberately used **existing recordings**, not more Gazebo runs. It compared failed `e01` with successful uncalibrated `p03` and `e05`, whose approach conditions were similar, and inspected MPPI source corresponding to the reported installed version (**1.3.12**).

### 9.1 Aligned observations

Set `τ = 0` when each run first enters the last **1.0 m**:

| Observation | `e01` spin | `p03` success | `e05` success |
|---|---:|---:|---:|
| MPPI yaw command at τ=0 | **+0.49** | +0.31 | +0.31 rad/s |
| Predicted yaw in next 1 s | **+26°** | +15° | +17° |
| Largest yaw command before 0.5 m entry | **+0.88** | +0.55 | +0.60 rad/s |
| Forward→reverse speed sign change | τ+2.5 s | τ+3.3 s | τ+3.2 s |
| Heading error reaches zero | at 0.64 m from goal | 0.20 m | 0.48 m |
| Yaw command on entering 0.5 m | **+0.58** | +0.44 | +0.37 rad/s |
| Reaches +1.0 rad/s | **Yes** | No | No |

The **first visible divergence was already present at the 1 m ring**, roughly 9 seconds before `e01` saturated. Its command was about **58% higher** than the two comparable successful runs. The stored state and path summaries did **not** explain why the optimizer initially chose the larger command.

The stronger discriminator across all 10 trials was **high angular command continuing away from the goal heading at the 0.5 m entry**, not simply crossing the desired heading outside 0.5 m. Other successful trials also crossed the heading outside that ring, but entered with smaller yaw commands.

### 9.2 Source/config facts reported in the addendum

The source check found the following behaviors in MPPI 1.3.12:

1. `GoalAngleCritic` is gated by distance to the **true goal position**, but scores angular error against the **last pose of the plan handed to MPPI**. That pose's yaw may differ from the final `NavigateToPose` yaw when the plan is truncated at a direction inversion.
2. The local path handler **truncates at the first position-based inversion**; it does **not** pass through all direction-change cusps. The later source-based detector predicted transformed-plan length in about **98%** of cycles for the compared runs. This supersedes earlier claims based on a heading-only cusp detector.
3. MPPI rollouts start angular dynamics from **measured angular velocity** and constrain stepwise angular acceleration (`az_max`). The reported code does **not** apply `wz_max` to each rollout state; it clips the **optimized control sequence** after updates. The sequence is reused/warm-started over iterations.
4. `ConstraintCritic` in the checked differential-drive implementation penalizes **linear** velocity violations, not yaw-rate violations.
5. With `regenerate_noises: false`, the sampled noise set is reused. The addendum traced the default random engine and inferred that fresh launches have identical noise samples, *assuming the same draw/reset history*. More precise wording than “MPPI is stochastic across runs” is: **the optimizer can be deterministic given identical state, plan, timing and random-state inputs, while the full simulated system still exhibits run-to-run variability**. The actual source of the initial divergence remains unknown.

These source facts sharpen the hypothesis, but they do **not** replace a full-controller trace of selected per-critic costs/control sequences.

### 9.3 Plausible command-saturation lock-in

Nominal parameters were:

\[
  |\omega| \le 1.0\ \mathrm{rad/s},\qquad
  |\dot\omega| \le 1.0\ \mathrm{rad/s^2},\qquad \Delta t=0.05\ \mathrm{s}.
\]

A measured rate of **1.079 rad/s** exceeds the 1.0 rad/s command bound. If a predicted state begins at this measured rate, a single step limited to 0.05 rad/s change cannot immediately enter the permitted speed interval (the earliest would be about **1.029 rad/s**). The addendum proposes that the rate-constrained rollouts, post-optimization control clipping, and warm start can form a **self-sustaining pinned-command state**.

The failure observation matches the proposed cycle:

```text
MPPI commands +1.0 rad/s
        ↓
Uncalibrated Gazebo plant rotates at about +1.079 rad/s
        ↓
Fused odometry reports +1.079 rad/s (> MPPI command limit)
        ↓
Rollouts start from out-of-bound measured speed
        ↓
Optimized sequence remains clipped near +1.0 rad/s
        ↺ possible self-sustaining loop
```

A **yaw-only replica**, based on the checked update mechanics and GoalAngle cost, reproduced the persistence from pinned starts at plant gains above approximately unity and release at/below unity. It also correctly distinguished a high-rate, wrong-direction terminal entry from a low-rate one in a simplified scenario.

**Limit:** That replica did **not** reproduce the real distribution of approaches or successes (it over-predicted failure). It supports the **sustaining mechanism**, not the **trigger**. This is a **moderate-confidence explanation**, not a demonstrated MPPI defect or universal threshold.

### 9.4 The critics did not visibly want to spin

The addendum evaluated the published `GoalAngleCritic` formula against seven recorded moments during the spin. For that critic alone, **braking** would have scored lower cost than the selected continued rotation. This argues against the simple statement “the goal-angle critic prefers spinning.”

But the analysis was **not a full MPPI cost replay** across all samples/critics, and it did not prove the braking alternative was reachable under the current sampled dynamics. The supported interpretation is narrower: **the optimizer selected sustained rotation despite a yaw objective that favors convergence**, consistent with a constraint/sampling/reuse problem.

## 10. Corrections to earlier interpretations

These corrections are important if the investigation becomes an article or presentation.

| Earlier or tempting statement | Corrected, evidence-sensitive statement |
|---|---|
| “The wheel mismatch caused every terminal failure.” | The mismatch is measured and corrected; it likely contributes to a particular saturation failure, but the calibrated stall remains unexplained. |
| “The fix solved B→A reliably.” | Initial 3/3 successes were followed by a calibrated stall and one unclassified non-completion in a separate batch. The later fixed-start instrumented batch achieved 5/5 successes, insufficient for a reliability claim. |
| “Uncalibrated always spins.” | Multiple uncalibrated runs succeeded; one of five instrumented ones spun. |
| “The global path was cropped incorrectly in `e01`.” | The transformed plan generally matched MPPI's source-defined inversion truncation; the older path-association issue came from a different experiment. |
| “MPPI follows paths through cusps.” | MPPI truncates the local path at a source-defined **position inversion**; a heading-only cusp detector produced false conclusions. |
| “Run-to-run variation proves fresh MPPI randomness.” | The recorded default noise generation was fixed/reused under specified conditions; other changing system inputs can produce differing outcomes. |
| “GoalAngleCritic favors continued spinning.” | Recorded single-critic counterfactual costs favor braking; full optimizer feasibility/costs remain unproven. |
| “Crossing the goal heading before the last 0.5 m is unique to failure.” | Some successes did so too; `e01` uniquely entered the ring with a substantial wrong-direction yaw command in this sample. |
| “A controller loop timing failure caused `e01`.” | The instrumented controller met its observed wall-time cadence in `e01`; timing cannot be ruled out generally, especially for startup/stall problems. |
| “0.92754 is our real robot's calibration.” | It is the effective multiplier for this **particular Gazebo contact model**. Hardware must be identified separately. |

## 11. Confidence and open-issue register

| Issue / conclusion | Current assessment | Why / next condition |
|---|---|---|
| Simulation steady-state yaw mismatch | **Measured, high confidence** | Independent +/- rate steps, IMU/ground truth and width tests. |
| Calibrated multiplier removes *that* steady mismatch | **Measured, high confidence for tested simulator** | Before/after yaw and odometry validation. |
| Uncalibrated MPPI can sustain angular saturation | **Measured** | Multiple recorded spins, including `e01`. |
| Measured > commanded yaw helping sustain the `e01` lock | **Supported explanation, moderate confidence** | Source mechanics + recorded feedback + reduced replica; not full-controller causal proof. |
| Why `e01` began turning faster than comparable successes | **Unresolved** | Earliest output divergence precedes saturation; recorded inputs do not discriminate. |
| Whether calibrated MPPI is immune to terminal spin | **Not established** | No spin in recent calibrated sample, but small N and transients remain possible. |
| Calibrated stall (`B2`) | **Unresolved, observed once with adequate GT but sparse commands** | Goal ~0.54 m away; later study did not reproduce it. |
| Planner goal loops / association on some routes | **Observed in earlier cases; scope limited** | Not shown as root cause of `e01`. |
| Variability in forward versus reverse terminal approach | **Observed; reason unknown** | Confounded with controller multiplier and execution history. |
| Startup failure under host load | **Observed; separate reliability work** | 4/14 instrumented launches failed before goal. |
| Robust real-world yaw response and safety | **Not tested** | No hardware measurements in the cited investigations. |

**Conditions that warrant reopening the internal MPPI investigation:** a calibrated run with a pinned yaw command or sustained spin; a recurrence of the calibrated stall with sufficient instrumentation; the same failure on hardware; or a failure that prevents meaningful mission progress after reasonable architectural mitigation.

## 12. Engineering lessons from the investigation

### 12.1 Separate plant identification from navigation tuning

Changing planner/controller gains would not by itself establish whether the robot's physical motion matched the assumed kinematics. The independent speed-step tests were decisive because they removed Nav2 from the loop. **Always compare commanded motion with independent body motion, not only encoder-derived odometry.**

### 12.2 A correct goal checker is not a convergence controller

`StoppedGoalChecker` was right to reject high-rate heading crossings. Relaxing tolerances or accepting a transient heading match would hide, not solve, the failure. Completing a goal requires **position, heading and stopped state together**.

### 12.3 Reverse capability and bidirectional planning are separate design decisions

A straight reverse movement demonstrated basic command-path compatibility. It did not establish that complex forward/reverse cusps were reliable. The robot may still benefit from reverse travel for real warehouse maneuvers, but normal cruise versus docking/undocking requirements should be explicitly defined.

### 12.4 Source inspection can correct an experimenter's interpretation

The source-based definition of a path inversion changed how the team interpreted earlier cusp statistics. Always validate analysis scripts against the implementation's **actual decision rule**, especially when the observed concept has an approximate geometric proxy.

### 12.5 Do not infer reliability from convenient small successes

Three calibrated successes initially looked like a complete repair; further runs found a stall. Ten fixed-start runs narrowed a spin mechanism but cannot establish a meaningful failure rate. We need **representative acceptance tests**, not unlimited experiments or selective before/after videos.

### 12.6 Keep investigations bounded and preserve uncertainty

After extensive simulation, offline comparison and source inspection were more productive than repeated poorly instrumented reruns. The team chose to end detailed MPPI root-cause work **without pretending all symptoms were explained**, and to spend the next engineering effort on an actionable controller architecture and system supervision.

### 12.7 Simulation performance and experiment integrity matter

During the instrumented runs, real-time factor was often **~0.58–0.70**. The controller's ~20 Hz wall cadence implied roughly ~30 Hz relative to the slower simulation clock, and the wheel plant lag spanned a different number of control updates than it would at real-time speed. The data did not identify this as the `e01` trigger, but the discrepancy is a limitation for real-hardware extrapolation. Control variable loading, startup readiness, reset state, bag finalization and disk partition availability must be explicit in test scripts.

## 13. Architecture decision: evaluate Rotation Shim without replacing everything

### 13.1 Why revisit the architecture?

The original plan used State Lattice + MPPI for feasible warehouse paths and bidirectional motion. However, the robot repeatedly struggled with final orientation. MPPI's near-goal command selection is a large optimization problem; a relatively simple rotational alignment phase is a plausible alternative for that **specific** task.

We therefore revisited two distinct options:

1. **Rotation Shim around the existing MPPI controller.** This can offer explicit pre-path/terminal rotational control while leaving MPPI responsible for ordinary path tracking.
2. **NavFn instead of State Lattice.** This would simplify global planning, but NavFn does not encode the same direction/orientation-feasible primitives and would change reverse/corridor behavior substantially.

**Decision:** Try **State Lattice + Rotation Shim + MPPI first**. Do not replace State Lattice with NavFn or switch to RPP solely because one MPPI spin occurred. A NavFn/forward-oriented route architecture remains an option if future warehouse requirements and a controlled functional comparison justify it.

### 13.2 Intended conceptual architecture

```text
Mission / NavigateToPose
          |
          v
    Smac State Lattice
          |
          v
  Rotation Shim wrapper
        /       \
       /         \
  selected yaw   ordinary path
  alignment      following
       |           |
       |           v
       |       MPPI DiffDrive
       \           /
        \         /
          v       v
       velocity conditioning
          |
     Collision Monitor
          |
    diff_drive_controller
          |
         Gazebo
```

This is a **conceptual** diagram of intended delegation, not a verified launch/configuration graph.

### 13.3 What we expect it might address

- **Terminal heading alignment once the robot is sufficiently close to the goal XY**, rather than asking MPPI alone to settle every large remaining angular error.
- **Initial heading/path alignment**, potentially reducing problematic large-angle entry conditions on some routes.
- A clearer, easier-to-observe split between ordinary path tracking and an explicit rotation action.

### 13.4 What it does *not* prove or automatically fix

- Rotation Shim **has not been tested on this AMR yet**; improvement is a hypothesis.
- It may **not activate early enough** to prevent a bad MPPI maneuver beginning outside final XY tolerance.
- It does not inherently explain or fix the **0.54 m calibrated stall**, far-field spin, raw global-planner loops, localization errors or startup failures.
- Its handoff to/from MPPI, behavior on direction-change cusps, and path-orientation semantics in reverse must be verified against **the installed Jazzy implementation and actual YAML**, not assumed from conceptual documentation.
- Any explicit rotation must remain subject to collision constraints and the robot's physical/safety limits, including footprint sweep in narrow aisles.

### 13.5 Configuration and implementation principles

- Keep a **reproducible baseline** (existing State Lattice + MPPI, multiplier 0.92754) that can be restored immediately.
- Add the shim **as a distinct, reversible configuration**, initially retaining the same global planner and goal checker.
- Inspect actual Rotation Shim plugin parameters, in particular final-goal rotation enabling, heading thresholds, `use_path_orientations`, rotation collision checking and the exact plugin nesting in the installed Nav2 version **before writing YAML**.
- Treat reverse-path compatibility as a real acceptance criterion; do not accidentally make the robot spin into the forward heading at a legitimate reverse segment.
- Do not globally weaken stopping or pose tolerances to make the shim appear successful.
- Do not claim the shim is a fix based on a single successful B→A run.

## 14. What “dependable navigation” must mean next

A good navigation subsystem must deliver **one of two acceptable outcomes**:

1. Reach the requested pose **inside XY/yaw tolerances with acceptable stopped motion**, and report genuine success; **or**
2. Detect inability to progress, **stop/cancel or transition to a bounded recovery safely**, and return/report a meaningful failure if recovery cannot succeed.

Indefinite spinning, indefinite goal-active stalling and false success are unacceptable. A recovery policy is **not** permission for routinely poor controller performance.

### 14.1 Functional criteria for the Rotation Shim evaluation

Initial *proposed* criteria, not yet approved test results:

| Behavior | Required observation |
|---|---|
| Ordinary B→A travel | Reach goal normally with active footprint/collision constraints |
| Large final heading error | Settle to goal yaw without repeated full turns or angular saturation lock-in |
| Reversing | Retain intended reverse motion and handle direction-change cusps without unexpected handoffs |
| Goal acceptance | Ground truth and Nav2 result consistent with XY **0.25 m** and yaw **0.20 rad**, plus stopped-velocity conditions |
| Stall/spin | Detect and bound the failure; do not leave a goal active indefinitely |
| Narrow aisle / footprint | No new unsafe in-place rotation caused by shim behavior |
| Architecture regression | No obvious degradation in normal travel, planner interface or docking-staging approach |

A **30-simulation-second terminal-convergence deadline after first entering 1 m** was previously discussed as an *initial screening rule*, motivated by the successful approach durations, **not as a final universal product requirement**. It must be adapted if a legitimate route, obstacle, or recovery needs more time. Separate wall-clock safety caps remain necessary because the simulation can run below real time.

### 14.2 Short, bounded evaluation plan

We deliberately do **not** want another sprawling research campaign. Proposed sequence:

1. **Repository audit, no modification.** Read installed Rotation Shim plugin API/YAML, existing `controller_server` configuration, saved MPPI baseline, reverse planner/cusp configuration, and current Git status. Confirm disk space on the actual recording partition.
2. **Reversible candidate setup.** Create the smallest isolated shim configuration change; do not modify physics, wheel calibration, goal checker or planner simultaneously. Document the exact diff and loaded runtime values.
3. **Small meaningful test set.** Normal B→A, a terminal large-heading correction, a reverse/cusp maneuver, and one constrained-space case if it can be run safely. Record whether the **shim or MPPI** produced commands at each phase, alongside actual pose/velocity and action result.
4. **Go/no-go review.** If terminal behavior improves **without** introducing reverse/collision problems, accept the shim as a candidate baseline and move to integration plus bounded supervision. If it does not, preserve the failure evidence and reassess rather than blindly tuning many parameters.

This initial screening is **not** a reliability certification. Later integrated mission validation must cover more varied starts, orientations, routes, obstacles and recovery conditions. No run count from the previous tiny datasets should be interpreted as demonstrating an acceptable field failure probability.

### 14.3 Navigation supervision remains required

Independent of shim success, the mission/controller-supervision layer should detect **persistent rotation without goal-error improvement, no-progress stalls, repeated failed recovery, and runaway active goals**. A measured yaw rate above `wz_max` can be a **diagnostic signal**, but should **not alone** trigger emergency cancellation because transient overshoot or different operating modes may be legitimate.

A supervised response should be stateful and bounded: detect -> check goal/task context -> command a controlled stop/cancel -> verify action termination and commanded/actual stopping -> recover if safe and permitted -> retry only a limited number of times -> report a fault if still unsuccessful. Real-hardware independent protective/safety functions are **separate** from Nav2's software collision monitor and mission watchdog.

### 14.4 Implementation ownership under the AI collaboration agreement

- **Project owner:** requirements and acceptance criteria; decide whether reverse navigation and final yaw semantics satisfy warehouse operations; personally design/write worthwhile mission supervision, action coordination, recovery state machine and fault policies.
- **Claude Code:** repository/API inspection, routine shim configuration and launch integration as delegated, run automation, logging/plotting, repeatable functional tests, and critical review of owner-written code.
- **ChatGPT:** discuss architecture, trade-offs and ROS 2 API concepts; review experiments and technical claims; help design the behavior before the owner implements core learning-value functionality.

This keeps the project moving **without turning every bug into a research program or outsourcing the owner's core engineering judgment**.

## 15. Current decisions and next-action checklist

### Decisions made (as of 25 October 2026)

1. **Retain** the empirically calibrated `wheel_separation_multiplier: 0.92754` for the **current simulation**, with the caveat that it must be reidentified on hardware.
2. **Do not claim** the calibrated baseline is already reliable or that the calibrated stall is solved.
3. **Pause** further deep MPPI root-cause experiments unless the issue recurs in calibrated or physical operation or blocks integrated navigation.
4. **Implement and evaluate Rotation Shim first** on top of State Lattice + MPPI; keep NavFn as an architectural alternative, **not the present change**.
5. **Require bounded supervision** of navigation failures as a separate layer; do not use it to excuse frequent controller failures.
6. **Preserve all previous results**, including unsuccessful and infrastructure-invalid experiments, and keep new experiments small and discriminating.

### Next engineering action

> **Inspect the current ROS 2 Jazzy Rotation Shim implementation and the repository's current controller-server configuration; propose a minimal, reversible change and a short functional validation matrix before editing code.**

The Rotation Shim implementation and test results will require **a new dated continuation record**. This document stops at the decision to pursue that work.

## 16. Source records and artifacts

Primary project-relative sources (paths as reported by the investigations):

| Source | What it establishes |
|---|---|
| `Warehouse_AMR_Terminal_Navigation_Investigation_2026-10-22.md` (companion evidence ZIP) | Chronology, geometry, system identification, before/after calibration and original path failures. |
| `calibration/nav_validation/reports/b_to_a_validation_report.md` | Three initial calibrated successes and uncalibrated control; test setup/caveats. |
| `calibration/yaw_rate_sysid/` and related geometry/`reports/` | Baseline and post-calibration rotational/linear tests and effective wheel-separation inference. |
| `Findings/Warehouse_AMR_BtoA_Behaviour_Variability_2026-10-23/` | Seven October 23 reruns with mixed start conditions, far-field/terminal spin and calibrated stall. |
| `investigations/btoa_variability_2026-10-23/report/Technical_Report_BtoA_Variability_Root_Cause_Investigation.md` | Ten instrumented B-start runs, outcome classification, full command/pose/planner evidence. |
| `investigations/btoa_variability_2026-10-23/report/Addendum_e01_vs_p03_e05_and_MPPI_1.3.12_source_check.md` | Offline alignment, MPPI source mechanics, yaw-only replica, corrections to earlier report and strongest remaining hypothesis. |
| `investigations/btoa_variability_2026-10-23/config/classification_criteria.yaml` | Predeclared offline SUCCESS/SPIN/STALL labels and thresholds. |
| `investigations/btoa_variability_2026-10-23/config/experiment.env` | Simulation/wall limits, watchdog recording overrun, settle time and disk-space cap. |
| `investigations/btoa_variability_2026-10-23/config/topics.txt` | Logged topic selection. |
| `investigations/btoa_variability_2026-10-23/config/watchdog_experiment.yaml` | Experimental early-stop thresholds; **not** the final mission watchdog design. |
| `investigations/btoa_variability_2026-10-23/analysis/summary_table.csv` / `.md` | Per-run outcomes, metrics and comparisons. |
| `investigations/btoa_variability_2026-10-23/analysis/terminal_entry_state.txt` | Entry pose/speed/yaw-command differences between approach families. |
| `investigations/btoa_variability_2026-10-23/analysis/addendum/` | Aligned comparisons, critic counterfactual calculations, replica results and archived MPPI 1.3.12 source files. |
| `investigations/btoa_variability_2026-10-23/runs/` | Original per-run rosbags, telemetry, runtime configs, provenance and launch logs (retained in the original repository). |
| `investigations/btoa_variability_2026-10-23/CHANGES.md` and `README.md` | Driver/experiment deviations, run commands and reproducibility instructions. |
| `CLAUDE.md` | Division of engineering responsibility and owner-learning workflow. |

Suggested figures already generated in the existing repository for future reports or videos:

- October 22: yaw command versus true yaw, optimal yaw trajectories, reduced-model replay, calibration ratios.
- October 23: `analysis/cmp_distance_rotation.png`, `analysis/cmp_first_plans.png`, `analysis/e01_m1.0/fig_timeline.png`, `fig_paths.png`, `fig_snapshots.png`, `fig_association.png`.
- October 24 addendum: `analysis/addendum/fig_addendum_aligned.png` and `fig_addendum_replica.png`.

The figure filenames refer to the **original project directories** and are not necessarily embedded alongside this standalone document.

## 17. Claims we can and cannot make publicly

**Supportable:** “We measured an approximately 7.8% yaw-response mismatch in a Gazebo differential-drive AMR, corrected the effective wheel separation, and observed substantially improved navigation in an initial comparison. Subsequent repeated tests revealed outcome variability and an unresolved calibrated stall. An instrumented uncalibrated spin showed MPPI output saturated while its odometry feedback exceeded the configured yaw limit. Source inspection identified a plausible constraint/sequence-reuse mechanism sustaining that spin.”

**Not supportable yet:** “We proved a universal Nav2 MPPI software bug,” “Rotation Shim fixes the problem,” “calibration eliminates all failures,” “the measured 20% spin fraction is the true field failure rate,” “the calibrated system is safe/reliable in production,” or “the entire problem was caused by State Lattice loops.”

For a technical article or demonstration video, show any **individual failed and successful runs as examples, not as deterministic before/after proof**. Explain the different initial conditions, unsuccessful calibration run, source corrections and unresolved stall. Such caveats strengthen engineering credibility rather than diminish the value of the investigation.

---

## Final engineering conclusion

The investigation progressed from “Nav2 keeps spinning” to a quantified, independently verified **simulator wheel-kinematic mismatch**, a better-instrumented understanding of **when MPPI selected an unbounded terminal rotation**, and a plausible **feedback/optimizer saturation mechanism**. It also showed why one successful test or one elegant explanation is insufficient: the uncalibrated AMR sometimes succeeded, and a calibrated AMR once stalled.

The team is **not choosing to ignore these failures**. Instead, it is moving from open-ended diagnosis to a controlled engineering change: **test Rotation Shim as an explicit terminal-orientation controller while retaining the currently calibrated State Lattice + MPPI system, then add robust navigation failure supervision and validate both ordinary and difficult maneuvers**. The next test must assess whether the change actually improves behavior, not assume it does.

**Status at document close: Rotation Shim approved for evaluation; not yet configured, executed or validated.**
