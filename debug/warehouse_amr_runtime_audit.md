# Warehouse AMR – Runtime + Rosbag Navigation Audit (read-only)

Bags: `debug/nav_problem_home_to_b` (70 s wall / 42.5 s sim) and `debug/nav_problem_b_to_a` (107 s wall / 66.7 s sim).
Method: both MCAP bags were decoded offline; every number below is computed from the bag. The live ROS graph was only read (`ros2 topic info`, `ros2 param get/dump`, one `topic echo --once`). Nothing in the repo, install tree or running system was modified; scratch scripts live in the session scratchpad.

Time convention: "t" = sim seconds relative to the first `/unsmoothed_plan` message of each bag (Home→B: sim 59.2555; B→A: sim 122.5645). Sim ran at RTF 0.58–0.69 (steady 0.62–0.63), so 1 sim-s ≈ 1.6 wall-s.

---------------------------------------------------------------------
## 1. Executive conclusion
---------------------------------------------------------------------

1. **The two bags do not show the same failure.** Home→B is a clean run: 49 plans, zero rotate-in-place poses, zero pure-rotation command ticks, arrival at 29.4 s with 2 cm / 1.9° error. Only a mild ±0.25 rad/s steering chatter is visible. The bag also starts ~1 s into an already-running navigation, so the beginning of that run (where an initial spin would occur) was not captured. B→A is the real failure.
2. **B→A fails in the last ~1.4 m before the Station A staging pose.** The robot cruises west fine (t=4–19), slows to a near-stop at 0.5 m/s→0 (t≈19–23), idles for ~8 s, then **orbits counter-clockwise at r≈0.29 m for the remaining 34 s: 1277° of yaw, 3.55 laps**, closest approach 0.46 m to the staging xy (tolerance 0.10 m). The bag ends mid-orbit, no abort or recovery seen.
3. **Why a ring exists at all (planner, expected):** the staging pose is `yaw = 0°` (+x), 0.70 m *before* the dock, but the robot arrives from B travelling in −x. With a forward-only lattice (`allow_reverse_expansion=false`, r_min = 0.5 m) the only legal solution is a forward U-turn near the goal. All 87 plans with |net yaw change| ≥ 60° have the **same sign (CCW)**. Smoothing preserves the ring (identical pose counts, same rotate-in-place runs, max deviation 0.30 m once, typically < 0.07 m). So the loop is a *valid, stable-in-direction* planner answer, not noise.
4. **Why the robot never finishes it (primary failure): MPPI does not execute or converge on it.**
   - It orbits at r≈0.29 m, tighter than the plan's 0.5 m arcs.
   - It keeps commanding CCW (99.1 % of orbit odom steps rotate CCW) even when the active plan is short and straight. Example t=53.5 s: pose (−0.79, −0.31), yaw +2° (goal heading is 0°), 0.13 m lateral error, active plan 0.93 m straight → `cmd_vel_nav` = (v 0.35, w **+0.83**); 0.5 s later the plan's lookahead says −20° (turn right) and w is still **+0.92**.
   - Commands almost never reverse after a replan (1 w-sign flip across 68 substantive replans; mean |Δw| 0.12 vs 0.09 for same-path replans).
5. **Replanning is a secondary amplifier:** once per lap the plan flips direct → loop when the start-heading bin reaches 45°/63° (4 times in the orbit), and flips loop → direct when it falls back to 27°/0°. 68 of 100 B→A replans are not "same path advanced". But because MPPI doesn't react within the 0.64 s replan period, the flips shape the limit cycle rather than cause it.
6. **The command pipeline is exonerated.** `cmd_vel_nav` ≈ `cmd_vel_smoothed` (max |Δw| 0.029 rad/s in B→A) ≈ `cmd_vel` (max |Δw| 0.024). No stale timestamps (age ≤ 0.04 s; all stamps in sim-time), no command gaps (> 0.1 s: 0 in B→A), Collision Monitor never published a state and never altered a command.
7. **Localization is consistent and not the driver:** fused `/odom` (= odom→base_footprint TF) agrees with the IMU gyro integral to ≤ 0.4°/s-window; plan start poses match the robot pose to ≤ 0.018 m and ≤ 13.4° (exactly the lattice-bin rounding). AMCL jitter is the same size in the benign Home→B run.

Confidence that the **planner's U-turn geometry (A) + MPPI near-goal non-convergence (C)** is the primary pair: HIGH for the pair, MEDIUM for MPPI's internal mechanism (see §9/§10).

---------------------------------------------------------------------
## 2. Home → B timeline (t relative to first plan, sim 59.2555)
---------------------------------------------------------------------

| t (s) | Event | Evidence |
|---|---|---|
| −0.26 | Bag starts mid-motion: `/cmd_vel` already v 0.25, w 0.20 (sim 59.00). `/cmd_vel_smoothed` and `/odom` first appear at +0.9 s (recorder start-up). Navigation start **not captured**; first-second mismatches between nav and smoothed are an artefact of that gap. | first stamps |
| 0–10 | Acceleration 0.30→0.52 m/s along the path, 16.3 m plan. Mild steering chatter: w ∈ [−0.13, +0.14], yaw 19°→10°→18°→15°→14° | `cmd_vel_nav`, odom |
| 10–13 | Gentle right turn (w ≈ −0.1 → −0.05), yaw 14°→0° | |
| 13–24 | Cruise 0.57→0.71 m/s, yaw ≈ 0°, w ≈ 0 | |
| 24–28 | Slight yaw bump (3–5°) and decel 0.68→0.38 m/s | |
| 28.5–29.4 | Final approach 0.36→0.13 m/s; last nav command at 29.38 | |
| 29.4 | **Arrival** at (8.74, 2.71), yaw 1.9°; goal (8.72, 2.70, 0°). Zero commands thereafter (13 s still) | |

Counts: 0 pure-rotation command ticks; 0 rotate-in-place runs in 49 raw plans; plan period 0.611 s sim (1.025 s wall); path lengths 16.3→0.15 m shrinking as the robot advances (37 of 48 transitions "same path advanced", 11 minor shape changes).

---------------------------------------------------------------------
## 3. B → A timeline (t relative to first plan, sim 122.5645; wall run = 106.6 s)
---------------------------------------------------------------------

| t (s) | Phase | Evidence |
|---|---|---|
| 0–3.6 | **Initial U-turn out of B (legit).** v 0→0.38, w −0.5…−0.9; yaw −24°→−171° (−157° in 5 s). Plan 0 starts with a −90° rotate-in-place (10 poses) then an arc heading west. Robot follows (lookahead error → 0 by t≈3). | plans 0–5 |
| 3.6–18 | **Cruise west** 0.5–0.63 m/s along y≈2.3→1.7, Dubins-like tails. Steering weave: 7 w-sign changes, peak |w| 0.38, detrended yaw pk-pk 30°. Plan start-yaw toggles −180°/−153° 7 times in 22 replans. | §4.4/§7 |
| 18.1 | Robot enters 1.4 m of the staging xy (the zone where MPPI's Goal/PathFollow thresholds change). | distance |
| 19–23 | Path now leads away from the goal (west/south loop). v falls 0.38→0.05 though lookahead errors are small. Plans 30–35 switch between direct/loop with initial rotations −45°, −72°, −53°, −27°. | a5 table |
| 23–31 | **Stall at (−0.59, 0.47), yaw ≈ 154–160°.** Near-idle (v < 0.05 and |w| < 0.12) for 3.0 s continuous (23.5–26.6); v ≤ 0.16 and mean |w| 0.20 over 22–32. w ramps 0→0.39 (t=28.2), collapses to 0.03 (t=29), ramps again. Plans 38–42 are identical (same 38 poses, robot not moving). Path lookahead error at that time +40…+72° (wants a CCW turn) but cmd ≈ 0. | plans 36–47 |
| 31–35 | Spin-up: w 0.3→0.57, v 0.03–0.14. | |
| 35–end (64.9) | **CCW orbit.** v mean 0.19, w mean 0.76 (v/w ≈ 0.29 m). Laps complete at t = 42.7, 50.7, 60.3 s (8–10 s/lap). Orbit bbox x[−1.15, −0.21], y[−0.32, 0.44]. Closest approach to goal 0.46 m at t=37.8 (−0.23, 0.14). | rm, cmd |
| 64.9 | Bag ends mid-orbit, robot (−0.42, 0.05), yaw 77°. No arrival, no abort, no docking hand-over visible. | |

Command-mode fractions (from `cmd_vel_nav`): t[0,4): 22 % pure-rot / 70 % arc; [4,19): 0 % / 33 % arc / 67 % straight; [19,23): 13 % / 40 % / 47 %; [23,31): 58 % pure-rot-ish / 42 % idle; [31,66): **7 % pure rotation, 89 % arcs** – i.e. the "excessive rotation" is a tight driven circle, not in-place spinning.

---------------------------------------------------------------------
## 4. Global planner analysis
---------------------------------------------------------------------

### 4.1 `/unsmoothed_plan` – per-bag summary

Home→B (49 plans): n_poses 314→4, length 16.34→0.15 m, start yaw alternates 27°/0° (lattice bins), end (8.72, 2.70, 0°), net tangent turn −30°…+4°, **0 rotate-in-place runs**, 0 loops/U-turns.

B→A (101 plans), start (8.74, 2.71) → goal **(0.10, −0.18, 0°)** in every plan (last plans (0.10–0.13, −0.14…−0.18)):

| Group | Count | Comment |
|---|---|---|
| Plans with |net yaw change| ≥ 150° ("loop/U-turn") | 64 | all positive (CCW) |
| "direct" (< 150°) | 35 | |
| "tiny" (< 0.3 m) | 2 | |
| Sign of every plan with |yaw change| ≥ 60° | **87 × positive, 0 × negative** | |

Selected plans (raw): idx, t, #poses, length, start→end yaw, in-place runs (poses idx range : yaw change)
- #0 t=0 207 poses 10.70 m, −27°→0°, [0-10: −90°]
- #36 t=23.25 46 poses 1.93 m, 153°→0°, [9-16: +63°]; pure-lattice ring, no analytic tail, yaw change +207°
- #40 t=25.8 38 poses 2.00 m, 153°→0°, [0-3: +27°] [10-14: +37°]
- #58 t=37.4 55 poses 2.25 m, 45°→0°, [0-2:+18°][11-15:+37°][26-36:+90°], yaw change +289° (first CCW 315° loop)
- #74 t=47.7 54 poses 2.10 m, 45°→0°, [0-10:+90°][24-34:+90°] (spin 90°, 0.5 m arc, spin 90°, straight)
- #85 t=54.7 60 poses 2.25 m, 63°→0°, [0-3][6-11][25-35], +274°
- #100 t=64.3 55 poses 2.77 m, 45°→0°, +298°

### 4.2 Rotate-in-place segments (consecutive poses < 1 mm apart with yaw change)

- Home→B: **0** runs in 49 plans.
- B→A: 105 runs (raw), distributed: start-of-path in the majority of loop plans (e.g. +90°, +45°, +27°, −27°), mid-path in plans such as #74/#85 (+90° runs at poses 24–35), none at the end. Total yaw per run 18°–90°, typically 3–11 poses per run. They are lattice zero-length primitives (the lattice file has 32 zero-length of its 112 primitives, 16 headings).
- The `/plan` (smoothed) has the same 105 runs, so smoothing neither adds nor removes them (it only changes the in-run yaw increments, e.g. +90°→+81°/+96°).

### 4.3 Loops, U-turns, large arcs

- Initial plans (t<18) contain a ~+115…+180° net yaw change because the goal heading is opposite the travel direction. The path goes west, then turns CCW near the goal; this is the whole-path signature of the forward-only U-turn.
- Final-metres loops (t > 19): length 1.6–2.8 m for a goal that is only 0.8–1.2 m away; bbox about 0.7×0.9 m; ring turning radius ≈ 0.5 m (lattice min).
- Ring does not require Dubins: plan #36 (+207°) has **no** analytic-expansion tail; #85 has a tail with 0° yaw change. Where a 7.07 cm-spaced tail (√2 × 5 cm; the analytic-expansion signature) is present, it averages 1.33 m (38 % of loop-plan length) and carries +95° of the turn (+61° in "direct" plans). So A3 (Dubins + no reverse) is a *contributor* to the shape of the last metre, not the origin of the ring. (Caveat: 7.07 cm also appears for 45° lattice moves; tails here have poses whose yaw does not equal the segment direction, e.g. segment −10.7° vs pose yaw 0°, which fits analytic-expansion output, but the planner source was not available to confirm.)

### 4.4 `/plan` vs `/unsmoothed_plan`

| Metric | Home→B | B→A |
|---|---|---|
| plans with identical stamp | 49/49 | 101/101 |
| pose count equal | 49/49 | 101/101 |
| length change (smooth − raw) | −0.009 mean (−0.03…0) | +0.009 mean (−0.114…+0.60) |
| Hausdorff raw→smooth (xy) | max 0.029 m | max 0.304 m (typ. 0.03–0.09) |
| start / end xy offset | 0 / 0 | 0 / 0 |
| net tangent turn diff | ≤ 3.2° | ≤ 13.5° |
| rotate-in-place runs raw / smooth | 0 / 0 | 105 / 105 |

Conclusion: smoothing **preserves** loops and rotate-in-place structure; it is not the source of the ring.

### 4.5 Replan-to-replan behaviour

- Period: 0.611 s (Home→B) / 0.644 s (B→A) sim = **1.03 s wall** (BT RateController runs on wall clock) → 1.55–1.64 Hz in sim time.
- Start pose follows the robot (≤ 0.018 m offset); start-yaw is the nearest lattice bin (mean error −0.5°, abs mean 5.9°, max 13.4°).

Classification of the 100 B→A transitions (same path = ≥ 90 % of new poses within 8 cm of the old polyline):

| Class | Count |
|---|---|
| same path shifted/advanced | 30 |
| loop → loop variant | 27 |
| other change | 17 |
| loop → different loop (changed rotate-in-place structure) | 15 |
| direct → loop | 6 |
| loop → direct | 5 |

Transitions with a sign-flipped initial rotation: #22 (−27→+27°), #28 (+18→−27°), #50 (−45→+53°), #58 (−27→+18°), #68 (−18→+72°), #74 (−27→+90°), #95 (−18→+72°). In the orbit the direct→loop flip happens **once per lap** at plans #58, #74, #85, #100 (start bins 45°/63°); loop→direct at #48, #67, #79, #94 (start bin ≈ −135°…−117°). Home→B: 37 same-path, 11 minor shape changes (start bin toggling 27°/0°).

Within the stalled phase (t 23–31) the planner emits near-identical plans (#38–#42 byte-for-byte), so the planner is *not* unstable there; the robot is the one not moving.

During the orbit the planner never requests a CW maneuver (CW yaw seen in 0.9 % of odom steps; no plan with yaw change ≤ −60° after t=19 except the near-straight #73/#99 at −27°).

---------------------------------------------------------------------
## 5. MPPI command analysis (`/cmd_vel_nav`, 988 / 2086 msgs, ~32 Hz sim = 20 Hz wall)
---------------------------------------------------------------------

- A. **Is MPPI itself commanding the rotation? Yes.** The downstream topics reproduce it exactly (§6). During the orbit MPPI emits v≈0.19, w≈0.76 (arcs, not in-place).
- Rate/gaps: Home→B median dt 0.030 s, one 0.18 s gap at the start; B→A median 0.032 s, max 0.036 s. Controller is on wall time (20 Hz wall = 32 Hz sim), whereas MPPI `model_dt = 0.05` → each tick advances 0.031 sim-s.
- Sign changes of w (|w| > 0.05): 11 per bag (Home→B: steering chatter; B→A: 2 per cruise weave, the rest at start/stall).
- Max |Δw| per tick 0.104 rad/s, max |Δv| per tick 0.047 → output is smooth; there is no tick-level dithering.
- **Command vs active plan** (B→A, t>33, lookahead 0.5 m):
  - On *loop* plans: sign(w) == sign(lookahead error) in **355/355** ticks (100 %); sign agreement with goal bearing only 63 %. MPPI follows the loop's turning direction.
  - On *direct* plans: lookahead agreement 293/393 (75 %); agreement with goal bearing 77 %. In 25 % of direct-plan ticks (the "pass-through" part of each lap) MPPI turns the wrong way.
  - On a rotate-in-place pose of the active plan: mean v 0.10 / w 0.51 (31 % of ticks v<0.05); on arcs/lines: v 0.20 / w 0.67. MPPI slows at in-place runs but does not stall on them: 1.0 s of data at the stall (t 23–31, nearest idx = 0 with yaw error −1…−4°, path wanting +27° then west) are the exception.
- Radius: median v/w = 0.29 m vs plan arcs of ≈ 0.5 m → MPPI turns ~1.7× tighter than the plan, with the same CCW sense.
- Per-lap pattern (e.g. t=46.0–55.5): w ramps 0.88→1.00 (wz_max) while yaw sweeps 17°→45° and the active direct plan's lookahead error goes +20°→−73°; then the plan flips to a loop (lookahead +91°) and w stays ≈1.0 → the command is continuous through the plan change.

---------------------------------------------------------------------
## 6. Velocity pipeline analysis
---------------------------------------------------------------------

| Quantity | Home→B | B→A |
|---|---|---|
| nav→smoothed max \|Δv\| / \|Δw\| | 0.092 / 0.368 (first 1.7 s only: recorder start-up gap) | **0.007 / 0.029** |
| smoothed→`/cmd_vel` max \|Δv\| / \|Δw\| | 0.002 / 0.002 | 0.004 / 0.024 |
| frac of ticks changed > 1e-3 (cmd vs smoothed) | 0.1 % | 0.1–0.2 % |
| `/cmd_vel` messages with no matching smoothed stamp | 36 (the pre-record commands 59.00–59.25) | 3 (121.95–121.99, w −0.33) |
| command age = rx-time − stamp, median / p95 / max | nav 0.000/0.001/0.076, smoothed 0.000/0.001/0.001, cmd 0.013/0.024/0.099 | nav 0.000/0.001/0.003, smoothed 0.000/0.001/0.003, cmd 0.014/0.025/0.039 |
| rx gaps > 0.1 s / > 0.2 s | nav 1/0; cmd 2/1 | 0 / 0 on all three |
| `/collision_monitor_state` messages | 0 | 0 (also nothing in 10 s live; publisher exists) |

Answers:
- B (smoother removes translation)? **No.** Smoothed ≈ nav.
- C (Collision Monitor modifies)? **No evidence**: `/cmd_vel` ≈ `/cmd_vel_smoothed`, state topic silent.
- D (missing commands)? **No** (only the 0.18 s start-up gap in Home→B).
- E (stale timestamps)? **No**: all stamps in sim time, age ≤ 0.04 s (B→A); `diff_drive_controller cmd_vel_timeout` is 0.5 s. The earlier TwistStamped problem is gone.
- F (w repeatedly changing sign)? Not in the orbit (0.9 % CW steps); sign changes occur in the cruise weave (7 per 14 s) and the start.
- G (command change after replan)? Weak: substantive replans → mean |Δw| 0.121 rad/s (0.088 in Home→B) vs same-path 0.094 (0.049); only 1 sign flip in 68 substantive replans. Cruise weave: corr(start-bin offset, next-0.5-s w) = −0.36 (B→A) / −0.68 (Home→B).
- The chain is a pure pass-through; the stamp and timeout parameters (`velocity_timeout 1.0`, `stamp_smoothed_velocity_with_smoothing_time true`, all nodes `use_sim_time=true`) are consistent with this.

---------------------------------------------------------------------
## 7. Localization analysis
---------------------------------------------------------------------

- `/odom` and the `odom→base_footprint` TF are identical (0.0000 yaw / 0.000 m difference): the fused odom is what the stack uses. `/diff_drive_controller/odom` (raw wheel) diverges from it: **wheel yaw under-reads by 33° over 1125° (≈ 3 %)** in B→A (0.57 rad end offset); fused-vs-wheel max twist-w diff 0.077 rad/s. Not an issue as long as the fused odom is used.
- IMU gyro integral vs fused-odom yaw in 1-s windows: max |err| 0.0038 rad (Home→B) and 0.0077 rad = 0.4° (B→A); correlation 0.9998 / 1.0000; total yaw B→A: odom 1140.4°, IMU 1140.4°, map-frame 1150.3°. No abrupt yaw jumps, no direction disagreement, no wheel/IMU disagreement in the fused path.
- map→odom (AMCL), 10 Hz sim:

| | Home→B | B→A |
|---|---|---|
| step xy median / p95 / max | 0.004 / 0.057 / 0.133 m | 0.011 / 0.061 / 0.133 m |
| step yaw median / p95 / max | 0.017° / 0.35° / 0.61° | 0.07° / 0.33° / 0.51° |
| net drift over run | 0.085 m, −0.7° | **1.00 m, +10.5°** (range yaw −3.1…+10.5°) |

  B→A detail: +0.66 m in the first 4 s (AMCL re-converging while turning out of B), net 0.31 m in 4–19 s, **net 1.42 m / +11.7° during the orbit** (sum of |steps| 4.7 m, i.e. mostly jitter plus a systematic share). Map-frame vs odom-frame yaw change differs by ≤ 1.0° per second.
- Correlation with planner changes: plan start poses are within 0.018 m of the robot's map pose and ≤ 13.4° of its yaw (bin rounding; mean −0.5°, so no systematic offset). The p95 jitter (6 cm) is the same in the clean Home→B run, so AMCL cannot explain why B→A alone circles. The only localization-linked effect: start-yaw rounding (±13°) toggles the start bin during cruise (Home→B: yaw sits at 14–19° vs bin midpoint 13.3°, giving 7 toggles in 17 replans).

---------------------------------------------------------------------
## 8. Comparison of the two runs
---------------------------------------------------------------------

Common to both:
- Plans replaced every 1.03 s wall (0.61–0.64 sim-s); MPPI → smoother → CM → diff-drive is a pass-through with correct stamps.
- Lattice start-yaw bin toggling and resulting mild steering chatter (Home→B: peak |w| 0.25, yaw pk-pk 11°; B→A cruise: peak 0.38, pk-pk 30°). Both pass through 26.6° bin boundaries.
- Smoothing preserves the raw plan shape.

Only in B→A / near Station A:
- Goal heading is opposite the travel direction → forward-only U-turn; 64 loop plans, 105 rotate-in-place runs, CCW only.
- The last 1.4 m: slow-down, 3-s near-idle (8 s total low-motion), then orbit, 3.55 laps.
- Direct↔loop flip once per lap.
- AMCL net drift 1 m / 10° (partly because there is much more rotation: 1140° vs 14°).

DockRobot involvement: all commands in B→A match the `controller_server` chain (`cmd_vel_nav`→`cmd_vel_smoothed`→`cmd_vel`); there are no docking-server commands (the 3 unmatched `/cmd_vel` messages are the start). So the bag only contains the **Nav2 staging navigation** (robot_executor sets `navigate_to_staging_pose = True`; staging = dock pose − 0.70 m along +x with yaw offset 0; `dock_prestaging_tolerance 0.50`). The precision docking controller never started, and `tag_station_a` appears in TF only once. The AprilTag is not involved, matching the report that turning perception off changed nothing.

Same mechanism? **Partly.** Common: lattice start-bin quantization produces plan-to-plan shape changes and mild steering chatter. Different: the large rotation is specific to the U-turn-at-goal geometry. The Home→B bag does not contain any excessive-rotation episode, so claims of an identical mechanism there cannot be made from this data.

---------------------------------------------------------------------
## 9. Root-cause attribution
---------------------------------------------------------------------

**A. PLANNER – major (necessary condition, not a defect by itself).** The staging pose faces +x, the robot arrives travelling −x, `allow_reverse_expansion=false`, turning radius 0.5, `rotation_penalty=2.0` → ring/U-turn plans of 1.6–2.8 m for a goal ≈ 1 m away, always CCW. Valid kinematically; unnatural for a mission that wants a short approach. The lattice also chooses a 315° CCW loop from 45° start rather than rotating CW in place about 70° (the bag has no costmap, so why it prefers that is **not** established; the kinematic lane-change margin dx ≈ 0.6 m vs 0.75 m in the "direct" case is a plausible reason).

**B. REPLANNING – secondary.** 68/100 B→A replans are not same-path; direct→loop / loop→direct flips occur once per lap; the 1 Hz default is wall-clock (0.64 sim-s). But identical consecutive plans in the stall (#38–42) and a smooth MPPI output show it does not initiate the failure. Only 1 w-sign flip across 68 substantive replans.

**C. CONTROLLER (MPPI) – primary execution failure.** (i) Stalls with v≈0 while the path asks for motion (t 22–31). (ii) Spins up then circles at r≈0.29 m (< 0.5 m), with 99 % CCW steps. (iii) Ignores a perfectly aligned straight plan (t=53.5 s) and keeps w ≈ +0.83…+0.92. (iv) Never gets closer than 0.46 m. Conditions where it happens: the robot is within 1.4 m of the goal xy (entered at t=18.1) with the path leading away from the goal and a forward-only vx_min=0. *Mechanism (not provable from the bag; my reading of Nav2's MPPI):* within `threshold_to_consider` of the goal, PathFollowCritic (1.4 m) and PathAngle/PathAlign (0.5 m) stop contributing, PathAlign (`offset_from_furthest=20` poses) needs ≥ 20 reached path poses (a 1–2 m lattice path has 25–55 poses, many duplicated by rotate-in-place), while GoalCritic (weight 5, 1.4 m) pulls trajectories toward the goal xy with no knowledge of the loop. This would explain "wait until facing the goal, then circle". The critic `.cpp` sources are not installed here, so this is inferred from the installed headers and parameters.

**D. COMMAND PIPELINE – not a cause.** Pass-through verified numerically (§6); no stale stamps, no gaps, no Collision Monitor action.

**E. LOCALIZATION – not a cause, minor contributor to jitter.** Consistent pose between planner and robot, IMU = fused odom, same AMCL jitter in the clean run. Start-bin rounding (±13°) adds plan-shape chatter. Large AMCL net drift in B→A (1 m/10°) tracks the 1140° of rotation, not the other way around.

Static-audit hypotheses:
- **B1 (MPPI on rotate-in-place segments):** partly supported. 105 runs exist; MPPI slows to v≈0.1 on them and spins up w; but the orbit is not tied to them (89 % of orbit ticks are arcs). The idle at t 23–31 coincides with a +27° initial run, so it is *possibly* related (LOW–MEDIUM).
- **B2 (1 Hz replanning flips maneuvers):** plans do flip (direct↔loop 11×, structure changes 15×) but the command does not follow quickly (MEDIUM as amplifier, LOW as root cause).
- **A3 (Dubins + no reverse → curved loops):** contributory. The ring appears without an analytic tail (#36); tail averages 38 % of loop length when present (LOW–MEDIUM).
- **B3 (localization shifting the start bin):** start bin tracks pose within rounding; no systematic offset (LOW).
- **B4 (simulation load/timing):** RTF 0.6 constant; controller and BT clocks are wall-based (20 Hz wall = 32 Hz sim; replans 1 Hz wall = 1.55 Hz sim); MPPI assumes 0.05 s per step but sim advances 0.031 s per tick. Can't explain the ring; may change effective MPPI dynamics (LOW).

---------------------------------------------------------------------
## 10. Ranked root causes
---------------------------------------------------------------------

| # | Cause | Evidence | Confidence |
|---|---|---|---|
| 1 | **Staging goal heading opposite to the arrival direction + forward-only r=0.5 lattice ⇒ mandatory forward U-turn in the last ~1.5 m** (ring, rotate-in-place primitives, CCW only). Home→B (same-direction arrival) shows none of it. | 87/87 big-turn plans CCW; 64 loop plans; `staging_yaw_offset 0`, staging x −0.70 | HIGH |
| 2 | **MPPI fails to execute/converge on that near-goal maneuver**: stalls ~8 s, then orbits at r≈0.29 m for 33 s, ignores aligned/straight plans, never within 0.46 m (tol 0.10 m). | commands, laps, t=53.5 example, 355/355 sign agreement on loops, 25 % disagreement on direct plans | HIGH that the controller is the primary execution failure; MEDIUM on the exact internal mechanism (critic gating) |
| 3 | **Plan instability once per lap** (direct↔loop at the 45°/63° start-bin, structure changes) keeps the target moving and sustains the limit cycle. | 11 mode flips, 15 structure changes, one per lap | MEDIUM |
| 4 | Lattice start-yaw bin quantization → steering chatter in cruise (both runs). | 7 toggles/22 replans, w sign changes 6–7, corr −0.36/−0.68 | MEDIUM (cause of chatter), LOW (as cause of the circling) |
| 5 | Dubins analytic tail shaping the last metre (A3) | tails 38–42 % of length, +61…+95° | LOW–MEDIUM |
| 6 | Rotate-in-place primitives confusing MPPI path-progress (B1) | MPPI slows on them; correlation with the stall only | LOW–MEDIUM |
| 7 | AMCL jitter / drift (up to 13 cm steps, 1 m net in B→A) | same jitter in clean run | LOW |
| 8 | Sim timing (RTF 0.6; wall-clock controller/BT) | | LOW |
| – | velocity_smoother / Collision Monitor / diff_drive / stale stamps | Pass-through verified | NOT SUPPORTED |

Live parameters checked (all consistent with the above): MPPI plugin `nav2_mppi_controller::MPPIController`, `motion_model DiffDrive`, `vx_min 0.0`, `vx_max 0.8`, `wz_max 1.0`, `PathAngleCritic.mode 2`, `PathAlignCritic.use_path_orientations true`, `controller_frequency 20`; critics Constraint 4.0 / Cost 3.81 / Goal 5.0 (thr 1.4) / GoalAngle 3.0 (thr 0.5) / PathAlign 14.0 (thr 0.5, offset 20) / PathFollow 5.0 (thr 1.4) / PathAngle 2.2 (thr 0.5); `time_steps 56, model_dt 0.05, batch 2000, iteration_count 1, temperature 0.3, ax 0.8, az 1.0`; planner `rotation_penalty 2.0`, `analytic_expansion_max_length 2.0`, `allow_reverse_expansion false`, lattice file `warehouse_diff_5cm_r0.5.json` (16 headings, 112 primitives, r=0.5); `velocity_smoother velocity_timeout 1.0`, `stamp_smoothed_velocity_with_smoothing_time true`; `diff_drive_controller cmd_vel_timeout 0.5`; `use_sim_time true` on controller, planner, smoother, BT navigator, collision monitor, smoother server, docking server, behavior server. `/cmd_vel_nav` has 6 publishers listed (controller_server + 5 behavior_server entries), 1 subscriber (velocity_smoother). `ros2 param dump` returned only 3 lines for collision_monitor and docking_server (their parameters were not readable as dumped); I did not read those two nodes' params individually.

---------------------------------------------------------------------
## 11. The single best next experiment
---------------------------------------------------------------------

Send **one** NavigateToPose from Station B to the **same staging xy (0.10, −0.18) with yaw 180° instead of 0°** (i.e. facing the direction of travel), leaving every parameter unchanged, and record the same topics.

Prediction if the analysis is right: no ring, no in-place runs, no stall, no orbit, arrival within tolerance in the same ~20 s as Home→B – this separates "forward-only U-turn at the goal" from a general MPPI/lattice defect with a single variable.
- If it still stalls/orbits within 1.4 m of the goal → the failure is MPPI's near-goal behaviour with these critics (cause #2), independent of the U-turn.
- If it succeeds → the correction is mission-level: approach Station A via a pre-staging waypoint in front of the staging pose (so the robot arrives already heading +x) instead of asking Smac to U-turn on the goal.

Limitations: no costmap/map/goal/BT/MPPI-critic data was recorded, so the planner's choice of a CCW loop at the 45° bin and MPPI's internal critic costs are inferred, not observed. The Home→B bag lacks the first ~1 s of a running navigation and shows no excessive rotation.
