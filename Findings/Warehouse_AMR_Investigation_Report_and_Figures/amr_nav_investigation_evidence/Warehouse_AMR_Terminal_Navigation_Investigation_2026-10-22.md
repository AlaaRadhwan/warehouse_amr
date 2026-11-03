# Warehouse AMR Navigation Investigation: Terminal Spinning, Path Association, and Kinematic Calibration

**Project:** Warehouse AMR / `warehouse_amr` (written under the earlier working name "MTS_AMR")  
**Investigation period:** September–22 October 2026 (principal instrumented tests: 20–22 October)  
**Software:** ROS 2 Jazzy; Nav2 Jazzy 1.3.12 (reported installed version); Gazebo simulation  
**Status:** Kinematic mismatch corrected and the original B→A route succeeded in three calibrated trials. Broader navigation reliability and the MPPI failure mechanism remain under investigation.  
**Purpose:** Preserve the experimental record, reasoning, limitations, and reproducibility information for future engineering work or a potential research paper.

> **Evidence convention.** **Observed** denotes behavior measured in rosbag, calibration, or simulation records; **source inspection** denotes conclusions reported from code/configuration inspection; **hypothesis** denotes an interpretation not directly established by controlled experiments. Numerical results come from the investigation reports and uploaded baseline datasets listed in §9. Some early test details are preserved from the 20 October status document and subsequent investigator summaries rather than independent raw-bag reanalysis for this report.

## 1. Executive summary

The AMR navigated normal route segments reasonably well but sometimes failed to converge to the requested final orientation. Failure modes included ~3 m State Lattice loops around nearby goals, incorrectly associated or truncated plans in MPPI, prolonged in-place spinning, and goal-checker non-completion. The behavior occurred during ordinary/staging navigation, without dependence on AprilTag precision docking.

Investigation initially focused on planner geometry and MPPI critic behavior. Enabling reverse motion solved a simple backward-driving test, **but did not eliminate** the B→A terminal spin. Detailed telemetry then revealed that a commanded yaw rate of approximately **−1.001 rad/s** produced an actual body yaw rate of **−1.079 rad/s** while wheel odometry still reported approximately −1.001 rad/s. MPPI's configured angular-speed limit was 1.0 rad/s.

Independent Gazebo ground-truth tests established a constant **1.07812 actual/commanded yaw-rate ratio** in both directions across six angular-speed setpoints. Straight-line tests showed no corresponding longitudinal scale error. The result identified a **0.640 m effective wheel separation** despite a 0.690 m physical wheel-center separation. Applying `wheel_separation_multiplier: 0.92754` to `diff_drive_controller` removed the angular-rate and fused-odometry discrepancies. Subsequently, the calibrated B→A mission succeeded **3/3 times**, with smooth heading convergence and without sustained saturation. A correctly launched uncalibrated control **failed 1/1 time**, locking into a similar high-rate spin, although it failed farther from the goal.

**Principal conclusion:** Correcting a reproducible simulated plant/controller kinematic mismatch substantially improved navigation and plausibly removed a major trigger for MPPI saturation. The experiments **do not prove** that the mismatch is the unique cause of all prior failures: cusp-truncated local goal orientations, path-association ambiguities, stochastic route selection, and localization error remain distinct concerns.

## 2. System and relevant configuration

### 2.1 Robot and navigation chain

- Differential-drive AMR; approximately **1.0 × 0.6 m** body, planning footprint approximately **1.0 × 0.74 m**; nominal driven wheel radius **0.100 m**, wheel-center separation **0.690 m**, wheel width **0.050 m**.
- Navigation: mission executor → `NavigateToPose` (or `DockRobot` for docking) → **Smac State Lattice** → **MPPI (DiffDrive)** → velocity smoother → Collision Monitor → `diff_drive_controller` → Gazebo. Precision AprilTag docking follows staging navigation and is a separate controller phase.
- State Lattice: 0.05 m grid; 16 headings; nominal curved primitive radius ~0.5 m; primitive file `warehouse_diff_5cm_r0.5.json` (reported 112 primitives: 80 forward arcs/straights and 32 in-place rotations). Reversing uses reverse expansion of supported primitives rather than native reverse primitives in that file.
- Differential wheel controller: `open_loop: false`, `position_feedback: true`; raw wheel odometry and IMU were independently available. Localization combined wheel and IMU information (EKF), with AMCL for map localization.

### 2.2 Parameters important to the final tests

| Area | Parameter | Value during final calibrated comparison |
|---|---|---:|
| State Lattice | `allow_reverse_expansion` | `true` |
| State Lattice | `reverse_penalty` / `rotation_penalty` | `2.0` / `2.0` |
| State Lattice | `analytic_expansion_max_length` | `2.0 m` |
| MPPI | `motion_model` / `vx_min` / `vx_max` | `DiffDrive` / `−0.35` / `0.8 m/s` |
| MPPI | `wz_max` / `az_max` / `model_dt` | `1.0 rad/s` / `1.0 rad/s²` / `0.05 s` |
| MPPI | `enforce_path_inversion` | `true` |
| MPPI | `inversion_xy_tolerance` / `inversion_yaw_tolerance` | `0.15 m` / `0.30 rad` (proposed baseline) |
| MPPI | `PathFollowCritic.threshold_to_consider` | `0.10 m` |
| Goal checker | `StoppedGoalChecker` XY / yaw | `0.25 m` / `0.20 rad` |
| Goal checker | Translation / rotation stopped thresholds | `0.05 m/s` / `0.10 rad/s`; `stateful: false` |
| Wheel controller | nominal `wheel_separation` / `wheel_radius` | `0.69 m` / `0.10 m` |
| Wheel controller | `wheel_separation_multiplier` | **`1.0` before; `0.92754` after** |

These values describe the investigation baseline, not a final physical-robot specification. Exact runtime values were checked for the calibration comparison; some earlier experiments relied on the installed YAML rather than a live parameter dump.

## 3. Investigation chronology and major experiments

| Stage | Change or test | Observation and conclusion |
|---|---|---|
| Early comparison | Smac2D + RPP; then State Lattice + RPP | Ordinary travel was possible, but terminal turning failures persisted; behavior occurred before precision docking. |
| Planner isolation | Display `/unsmoothed_plan` alongside `/plan` | ~3 m loops already existed in raw State Lattice plans; **the Smac smoother did not create them**. |
| Planner parameters | Analytic expansion max length `3.0 → 2.0`; rotation penalty `5.0 → 2.0` | Neither isolated adjustment eliminated terminal loops or stalls. |
| Command pipeline | Enable velocity smoother `stamp_smoothed_velocity_with_smoothing_time: true` | Corrected a genuine stale `TwistStamped` timestamp issue (~0.8–0.98 s old versus 0.5 s timeout); later spins originated in MPPI output. |
| MPPI near-goal behavior | Lower `PathFollowCritic.threshold_to_consider` `1.4 → 0.10` | Approach and yaw at arrival improved; terminal convergence still sometimes failed. |
| Yaw-π test | B→A staging goal changed temporarily to arrival-aligned yaw π | Approach improved but a nearby terminal error still provoked a ~3 m loop and failure; an opposing final heading was **not the sole cause**. |
| Path visualization | Record `/transformed_global_plan` and MPPI trajectories | Found endpoint association/collapse and truncation on goal-coincident loops (§4.1). |
| Bidirectional milestone | Enable State Lattice reverse, MPPI negative X velocity and cusp handling; switch to `StoppedGoalChecker` | A simple straight-reverse test completed successfully. The full B→A mission **still spun**, demonstrating that forward-only constraints were not the whole explanation. |
| Offline spin analysis | Compare MPPI command, wheel odometry, IMU/fused odometry and predicted trajectory | Identified persistent `wz_max` saturation and ~7.8% yaw-rate mismatch (§4.2). |
| System identification | Six open-loop commanded yaw-rate steps and four straight-line steps, with Nav2 inactive | Isolated a constant rotational scale error but no longitudinal error (§5). |
| Calibration and controlled retry | Set wheel-separation multiplier `1.0 → 0.92754`, repeat calibration and B→A | Rotational mismatch disappeared; B→A succeeded 3/3 calibrated trials, versus sustained spin in one valid uncalibrated comparison (§6). |

A high Gazebo physics rate (initially 2 kHz, from `max_step_size=0.0005`) was also identified as a possible simulation-load concern, **not** an explanation for raw-planner loops. The actual physics rate used for every later run is not established by the consolidated evidence and should be reported explicitly in a future experimental protocol.

## 4. Failure characterization before calibration

### 4.1 Planner loops and MPPI transformed-path association

The raw State Lattice output repeatedly generated approximately **3.0–3.3 m goal-coincident loops** even when the robot was already within a few centimetres of the requested XY. The 0.5 m lattice turning radius did not forbid in-place rotation: the primitive set explicitly included it. Why the planner sometimes chose a large loop instead of a short heading correction was not resolved.

In a forward-only yaw-π visualization run, the investigator reported:

- `/transformed_global_plan` recorded **1,288** messages; **95** contained five poses or fewer, across seven episodes.
- At one instant a **61-pose, 3.23 m** global loop was reduced to its last three poses, approximately **0.14 m** long, because the robot near the goal was associated with the end of a loop whose beginning and end coincided geometrically.
- With `prune_distance = 2.5 m`, the 3.0–3.3 m loop could not be fully represented in the transformed window. During one terminal interval, **434/508** windows ended >0.2 m from the true goal, with median endpoint XY offset **0.53 m** and yaw offset approximately **60°**.
- The optimizer's selected turn direction consistently disagreed with the initial rotation direction of those long local path segments, although it was consistent with its own selected trajectories.

These measurements support **path representation/association ambiguity** but do not establish why MPPI chose the opposing angular direction. Increasing `max_robot_pose_search_dist` alone was not expected to repair a loop whose nearest pose was its own endpoint. This is separate from the later kinematic finding; neither mechanism should be silently substituted for the other.

### 4.2 Bidirectional B→A failure and sustained spin

Reverse capability was enabled end-to-end (`allow_reverse_expansion`, MPPI `vx_min < 0`, inversion enforcement and downstream reverse velocity limits). A straight reverse maneuver completed with roughly **6.6 cm X error, 0.5 cm Y error and −0.15° yaw**. This verified that reverse paths and negative velocity were executable, but **did not test a direction-change cusp**.

The subsequent full B→A test failed at goal pose **(0.099, −0.175), yaw 0**. At cancellation the robot was about **3 cm** from the goal XY and approximately **156°** from its intended yaw. Reverse velocity had been used in **1,010/3,799** controller ticks (~27%; minimum about **−0.30 m/s**), so inability to reverse was not the cause.

Offline analysis of the recorded terminal phase found:

| Quantity | Pre-calibration result |
|---|---:|
| Distance from true goal XY during sustained spin | Mean `0.032 m`; maximum `0.059 m` |
| MPPI commanded angular rate, stable interval | `−1.0010 ± 0.0009 rad/s` |
| Raw wheel odometry angular rate | `−1.0012 rad/s` |
| IMU gyro and fused `/odom` angular rate | approximately `−1.0791 rad/s` |
| Spin lock-in | ~wall time `118.8 s` |
| Heading crossings | Nine crossings of true goal yaw, still at full-rate command |
| Near-heading samples | 135 cycles with absolute yaw error <15° still commanded ~−1.0 rad/s |
| Selected MPPI trajectory horizon | ~2.8 s; predicted ~−121° rotation on average |
| Transformed local plan | One pose in **39%** of spin cycles; otherwise 2–12 poses |

The command was already saturated at the MPPI output: smoother and collision-monitor outputs agreed with it. A `StoppedGoalChecker` correctly **refused success** because angular speed never fell below its `0.10 rad/s` stopping threshold. Under a non-stopped goal checker, momentary tolerance crossings could otherwise look like completion.

The full global plan retained the requested yaw `0°` in **370/370** examined raw/smoothed plans. The transformed plan used by MPPI was less stable: its endpoint yaw varied with replanning and cusp truncation, and **988/1,010** multi-pose messages coincided with a cusp in the recent global plan. As reported for Jazzy 1.3.12, MPPI's `GoalAngleCritic` uses the **last transformed-path pose's yaw**, not necessarily the true navigation goal yaw. This is an additional potential obstacle to settling even when kinematics are correct.

**Data caveat:** The older bidirectional failure bag lacked `/tf`, action feedback/status, local costmap and per-critic costs. Some coordinate transforms were reconstructed; the above figures should not be mistaken for a complete per-critic causal attribution. Report time was **wall time**, not simulation time.

### 4.3 Proposed MPPI saturation mechanism — supported hypothesis, not proof

For the recorded MPPI settings:

$$
\omega_{\max}=1.0\;\mathrm{rad/s},\quad a_{\omega,\max}=1.0\;\mathrm{rad/s^2},\quad \Delta t=0.05\;\mathrm{s}
$$

The nominal maximum one-step angular-speed change is **0.05 rad/s**. Starting from feedback of `−1.079 rad/s`, the least-negative next speed under that acceleration bound is approximately `−1.029 rad/s`, still outside the configured `−1.0 rad/s` speed limit. The measured state thus creates a **one-step constraint inconsistency** if both bounds are treated as hard constraints on the next predicted speed.

A simplified yaw-only reproduction of the reported MPPI update rules (not full Nav2, and not measured per-critic costs) reproduced persistent negative-speed saturation when actual angular response was 1.08× commanded; it generally converged with matched dynamics and a fixed yaw target. Replaying changing cusp endpoint yaw targets made settling substantially worse even without the gain mismatch. This supports a plausible feedback/saturation mechanism but **does not prove the complete optimizer must lock**, nor identify a universal percentage-error threshold. The robot remained physically capable of braking; the concern is the optimizer's behavior in this out-of-bounds state.

## 5. Independent kinematic system identification

The navigation stack was removed from the experiment. Known steady velocity commands were sent to the wheel controller and compared with Gazebo **ground-truth pose**, raw wheel odometry, `/joint_states`, IMU and EKF-fused odometry. The wheel controller used **position feedback**, not command-derived open-loop odometry. Ramps were used; speed ratios were estimated from steady portions. All initial six yaw steps and four linear steps completed cleanly.

### 5.1 Rotation baseline (multiplier = 1.0)

| Command `ω` (rad/s) | Gazebo true `ω` (rad/s) | Raw wheel odom (rad/s) | IMU gyro (rad/s) |
|---:|---:|---:|---:|
| +0.40 | +0.4312 | +0.4000 | +0.4312 |
| −0.40 | −0.4312 | −0.4000 | −0.4312 |
| +0.70 | +0.7547 | +0.7000 | +0.7547 |
| −0.70 | −0.7547 | −0.7000 | −0.7547 |
| +1.00 | +1.0781 | +1.0000 | +1.0781 |
| −1.00 | −1.0781 | −1.0000 | −1.0781 |

The average ratio `ω_true / ω_command = 1.07812`, constant to the reported precision across direction and speed. Raw wheel odometry/ground truth was about **0.92759**; IMU/ground truth approximately **1.00000**. The fused yaw-rate twist agreed with the IMU, but its integrated pose-yaw slope was only **0.98205** of ground truth.

### 5.2 Translation baseline and parameter identification

At **+0.30, −0.30, +0.60 and −0.60 m/s**, Gazebo ground truth, wheel-derived speed, raw odometry and fused longitudinal speed all agreed with the respective command to the reported precision (`k_v = 1.00000`); no meaningful lateral/yaw drift was observed. Consequently, the effective radius was consistent with the nominal **0.100 m**, whereas the rotational error reflected effective wheel separation.

For driven wheel angular speeds `φ̇_R, φ̇_L`, differential-drive kinematics give:

$$
v=\frac{r}{2}(\dot\phi_R+\dot\phi_L), \qquad
\omega=\frac{r}{b}(\dot\phi_R-\dot\phi_L).
$$

With `b_nominal = 0.69 m`, `k_v ≈ 1`, and `k_ω = 1.07812`:

$$
r_{\mathrm{effective}}\approx 0.10\,k_v=0.100\;\mathrm{m},\qquad
b_{\mathrm{effective}}\approx \frac{0.69\,k_v}{k_\omega}=0.640\;\mathrm{m}.
$$

Gazebo link-pose inspection placed the **wheel joint centers at 0.690 m**, as did the URDF, and the wheel collision cylinders had **0.05 m width**. Auxiliary scratch-model runs that varied *only* collision-cylinder width reported:

| Wheel width | Identified effective separation |
|---:|---:|
| 0.10 m | 0.5900 m |
| 0.05 m (baseline) | 0.6400 m |
| 0.02 m | 0.6723 m |

The near relationship `b_effective ≈ b_centers − wheel_width` suggests a **Gazebo wheel-ground contact effect**, not an incorrect nominal wheel-center dimension. Contact points were **not directly observed**; the 0.02 m run deviated slightly from the simple relationship. Thus the effective separation is an **empirical property of this simulated contact model**, not a reason to change the real wheel locations or assume the physical robot's separation is 0.640 m.

## 6. Calibration intervention and B→A validation

### 6.1 Single-parameter correction

The only intended kinematic intervention was in `src/amr_control/config/controllers.yaml`:

```diff
 wheel_separation: 0.69
 wheel_radius: 0.10
-wheel_separation_multiplier: 1.0
+wheel_separation_multiplier: 0.92754
```

Effective controller separation becomes `0.69 × 0.92754 ≈ 0.640 m`. URDF, MPPI, State Lattice, EKF, localization and goal checker were left unchanged for this comparison. The parameter was verified **at process launch**. Changing it with `ros2 param set` during a running controller had earlier proved insufficient to change actual controller behavior and that trial was excluded as an invalid control.

### 6.2 Before/after steady-state verification

| Measurement | Before | After calibration |
|---|---:|---:|
| True yaw rate / commanded yaw rate | `1.07812` | `1.00000` |
| Raw wheel odom yaw rate / Gazebo truth | `0.92759` | `1.00002` (worst error `0.007%`) |
| EKF fused pose-yaw slope / Gazebo truth | `0.98205` | `1.00000` |
| True longitudinal speed / commanded speed | `1.00000` | `1.00000` |
| Raw odom longitudinal speed / truth | `1.00002` | `1.00001` |

The requested validation limits (less than **0.5%** yaw-rate error and **1%** longitudinal error in steady tests) were met. The previously observed **1.8% EKF integrated-yaw shortfall disappeared**. This supports the explanation that the EKF had been combining correct IMU rotation with underestimating wheel-derived rotation.

### 6.3 Navigation comparison: same B→A staging goal

Procedure: full `Home → Station B → Station A` setup using `amr_bringup simulation_navigation.launch.py`, with B→A `NavigateToPose` goal **(0.099, −0.175, yaw 0)**. The calibrated and valid uncalibrated attempts used the same procedure/configuration except for the wheel-separation multiplier. Ground truth, control commands, planning topics, localization and action result were recorded.

| Trial | Multiplier | Result | Sim duration | Maximum commanded `|ω|` | Final true XY error | Final true yaw error |
|---|---:|---|---:|---:|---:|---:|
| Calibrated 1 | 0.92754 | **Succeeded** | 47.3 s | 0.43 rad/s | 0.112 m | −4.2° |
| Calibrated 2 | 0.92754 | **Succeeded** | 56.5 s | 0.43 rad/s | 0.015 m | −2.6° |
| Calibrated 3 | 0.92754 | **Succeeded** | 60.6 s | 0.43 rad/s | 0.057 m | +0.5° |
| Uncalibrated control | 1.0 | **Spin; operator stopped** | 69 s to stop | 1.00 rad/s (pinned) | Not reached; ≥5 m away | N/A |

**Calibrated terminal behavior:** no sustained spin; measured yaw speed remained below ~0.38 rad/s near completion. Run 1 corrected about **171°** in ~15 s and settled; runs 2–3 approached partly in reverse and required smaller final turns. Goal acceptance occurred at ground-truth linear speed **0.002–0.031 m/s** and angular speed magnitude **0.028–0.060 rad/s**, meeting the stopped-velocity requirements. The global plans included two forward/reverse cusps, but the transformed local plans showed **no cusp within the final metre** in these three successes.

**Uncalibrated control:** the multiplier `1.0` was actually loaded from a scratch controller configuration. MPPI saturated at approximately 1.0 rad/s, while true yaw speed rose to roughly 1.058–1.08 rad/s. The robot made **~3,491° (9.7 turns)**, and the run was stopped after a prolonged spin **7.7 m from the goal**, never entering the last 5 m. This shares the distinctive saturation signature with the earlier terminal failure **but not its location**.

The calibrated sample size is **three successes**; there is **one valid contemporary uncalibrated control** plus earlier pre-calibration failures. Route choices varied across calibrated runs. This is strong intervention evidence, **not a statistically sufficient demonstration of general reliability or proof of a unique root cause**.

## 7. Conclusions by evidence strength

### Established observations

1. Raw State Lattice plans could generate large goal-coincident loops before smoothing; MPPI transformed-path association could collapse/truncate looped paths.
2. Enabling reverse made backward driving functional but did **not by itself** resolve the terminal spin.
3. Pre-calibration yaw-rate feedback showed a reproducible **~7.8% body-versus-command discrepancy**; it was corroborated by independent Gazebo ground truth and matched by the IMU, not raw wheel odometry.
4. Straight-line calibration isolated that discrepancy to the **effective rotational geometry**, not the nominal wheel rolling radius, in the tested simulator.
5. Adjusting only the wheel-separation multiplier removed the steady yaw-rate disagreement and corrected EKF yaw-slope behavior; the tested B→A route then succeeded 3/3 times without sustained spin.
6. The stopped goal checker avoided declaring completion during high-rate rotation; it does not itself produce a convergent heading controller.

### Strongly supported, not conclusively demonstrated

- **Kinematic mismatch as a major trigger of MPPI saturation.** The before/after intervention, contemporaneous control, logged out-of-range velocity and reduced-model reproduction support this causal explanation. Full MPPI optimization dynamics and per-critic costs were not directly identified, however.
- **A simulation contact artifact as the origin of the effective 0.640 m separation.** The width sweep is persuasive; contact geometry was inferred, not observed directly.

### Still unresolved

- Under what exact conditions the full MPPI implementation becomes trapped in saturated angular motion when measured speed exceeds its nominal bound; whether it can reliably decelerate from out-of-bound feedback states.
- Whether cusp-truncated goal orientations and nearest-path association can independently cause terminal failures on other geometries after kinematic calibration.
- The planner's preference for ~3 m terminal loops despite in-place-rotation primitives.
- AMCL map-position error (up to **0.18 m**, means ~0.03–0.05 m in calibrated runs) and its implications for docking staging.
- Repeated-route reliability, rear-obstacle protection during reverse, and transfer of these findings to real hardware.

## 8. Implications, limitations, and recommended next work

**Engineering implication.** The correct physical geometry, the effective wheel-ground kinematics and the controller's assumed parameters need not coincide. Encoder feedback alone can appear self-consistent while actual body motion disagrees. Independent ground truth (or independent physical measurements) is essential for identifying such errors. Calibration improves nominal fidelity; it does **not** establish robustness to parameter variation, slip or sensor failures.

**Primary methodological limitations:** small navigation sample; stochastic MPPI/planner trajectories; no complete per-critic cost logging; missing TF/action records and reconstructed transforms in one older bag; different failure locations in near-goal versus uncalibrated-control spins; and tests largely confined to one map/goal geometry. Simulated contact-model estimates must not be transferred numerically to the physical AMR without fresh identification. Some recorded timestamps are wall time rather than simulation time; comparisons require explicit clock/frame alignment.

**Next tests, in priority order:**

1. **Nominal reliability matrix:** repeat B→A yaw 0 and yaw π, orientation-only goals, direction-change cusps, and normal cruise, with recorded action outcomes, ground-truth terminal error, accumulated rotation, local path endpoints and run time. Use the **calibrated** robot throughout.
2. **Path-association diagnosis:** deliberately exercise nearby, goal-coincident loops after calibration. Record `/plan`, `/unsmoothed_plan`, `/transformed_global_plan`, `/optimal_trajectory`, pose/TF and—if instrumented—critic costs. Establish whether changing local endpoint yaw independently destabilizes convergence.
3. **Out-of-bounds speed robustness:** vary controlled angular-response gain/parameter mismatch in both directions and at different commanded speeds. Measure not just success but recovery/stop behavior after MPPI receives `|ω_measured| > wz_max`. Do not mistake a few passing trials for a certified margin.
4. **Failure supervision:** strengthen spin/stall detection throughout the route, not just in the last metre, while allowing legitimate long heading corrections; add controlled stop/cancel behavior.
5. **Localization and docking:** characterize AMCL versus ground truth and then validate staging-to-AprilTag precision docking. Reidentify wheel parameters and sensor response on the physical robot.

**Potential paper framing:** a case study of **model–plant angular-velocity mismatch interacting with constrained stochastic predictive control**, supported by a controlled system-identification intervention. Useful stronger evidence would be more randomized calibrated/uncalibrated paired runs, full optimizer/critic instrumentation, controlled mismatch sweeps, and a quantified recovery strategy. Do not yet claim a general MPPI software defect or a guaranteed failure threshold.

## 9. Evidence inventory and reproducibility notes

The following paths are **paths within the original `warehouse_amr` workspace**, not automatically included in this report. Preserve the logs, source revision/commit, YAML snapshots, Gazebo and Nav2 versions, simulator seed and system clock settings before preparing a publication.

| Record | Original location / uploaded counterpart |
|---|---|
| Investigation baseline and earlier tests | `Warehouse_AMR_Navigation_Investigation_Status_2026-10-20.md` (uploaded copy with `(1)`) |
| Pre-calibration B→A spin analysis | `debug/nav_test_b_to_a_bidirectional/` and associated `REPORT.md` |
| Rotational system identification | `calibration/yaw_rate_sysid/runs/20261022_102121_baseline/` (uploaded `20261022_102121_baseline.zip`) |
| Linear system identification | `calibration/yaw_rate_sysid/runs/20261022_123234_linear/` (uploaded `20261022_123234_linear.zip`) |
| Kinematic geometry/width sweep | `calibration/yaw_rate_sysid/geometry/`, `runs_diagnostic/`, `reports/kinematic_identification_report.md` (reported; not independently included in uploaded ZIPs) |
| Calibrated rotation/linear validation | `calibration/yaw_rate_sysid/runs_after/20261022_124634_rotation_after/` and `20261022_124931_linear_after/` |
| Before/after aggregate table | `calibration/yaw_rate_sysid/reports/before_after_calibration.md` (uploaded) |
| Navigation comparison | `calibration/nav_validation/runs/`, `reports/b_to_a_validation_report.md` (uploaded) |
| Reproduction/analysis scripts | `calibration/nav_validation/run_b_to_a.sh`, `nav_goal.py`, `analyze_b_to_a.py`, `analyze_partial_run.py`, `kill_sim.sh` |
| Diagnostic figures | `fig1_yaw_error_and_angular_velocity.png`, `fig2_optimal_trajectory_vs_yaw.png`, `fig3_reduced_model_replay.png` (see companion evidence bundle) |

**Important data hygiene notes:** An initial uncalibrated control using runtime `ros2 param set` did not actually change the controller's active kinematic behavior and was **excluded**. Some preliminary tests inadvertently mixed orphaned Gazebo processes; those were marked superseded and rerun. A large `/trajectories` bag was ~3.2 GB, so retain its provenance and verify backups before deleting it. No changes were committed at the time of the cited reports; the final publication needs a frozen repository revision and reproducible environment manifest.

---

**Summary statement for later citation:** In a ROS 2 Jazzy differential-drive AMR simulated in Gazebo, a consistent 7.8% angular-response mismatch was measured despite nominally correct wheel geometry and accurate straight-line motion. Correcting the wheel controller's effective separation restored angular-rate and fused-yaw consistency and coincided with three successful formerly problematic navigation runs, while an uncalibrated control still exhibited saturation. The evidence supports a model–plant mismatch contributing to MPPI spinning but leaves path-association effects and robustness under uncertainty open.
