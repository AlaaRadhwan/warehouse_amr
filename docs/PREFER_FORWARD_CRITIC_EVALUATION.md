# MPPI PreferForwardCritic with reversing enabled — EXPERIMENTAL evaluation

Branch `experiment/navfn-mppi-evaluation`. Simulation only; Nav2 / `nav2_mppi_controller` **1.3.12** (Jazzy). Single runs (n=1 per condition) — no reliability claim.
Labels: **[observed]** measured (Gazebo ground truth + `/cmd_vel_nav`), **[source]** read in 1.3.12 source/config, **[inferred]**, **[open]**.
Metrics are GT-based (`scripts/analyze_goal.py`, `scripts/reverse_segments.py`): a reverse sample is a 0.2 s GT step moving opposite to body heading at >0.03 m/s; samples ≤1 s apart are merged into one segment.

## 1. Exact experimental configuration
Launch: `ros2 launch amr_bringup simulation_navigation.launch.py planner:=navfn structured_routing:=true mppi_path_interpretation:=2d_prefer_forward`
(overlays `config/profiles/mppi_2d_paths.yaml` + `config/profiles/mppi_prefer_forward.yaml`; defaults unchanged, existing profiles untouched). Values read back from the running `/controller_server`:
```
vx_min: -0.35   enforce_path_inversion: false   PathAlignCritic.use_path_orientations: false   PathAngleCritic.mode: 0
critics: [Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle, PreferForward]
PreferForwardCritic: enabled true, cost_power 1, cost_weight 5.0, threshold_to_consider 0.5
```
No other critic weight, horizon (56 × 0.05 s = 2.8 s), limits, costmap, footprint, calibration or goal tolerance was changed.

## 2. What the critic does [source, 1.3.12 `prefer_forward_critic.cpp`]
`cost[traj] += weight · Σ_t max(−vx_t, 0)·model_dt` (power 1; `power>1` raises the weighted sum to that power). It is skipped when the robot is within `threshold_to_consider` (0.5 m) of `data.goal`, i.e. the **last path pose**; no yaw terms, no geometry, no awareness of why reversing is chosen.
Consequence: the penalty is *proportional to reverse distance inside the horizon*, capped at 0.35 m/s·2.8 s·5 = **4.9 cost units** per trajectory. Near the goal it is off. How 4.9 compares to the other critics' typical magnitudes was **not measured [open]**.

## 3. Results (GT)
| Scenario | Config | Result | Dur s | Max/mean dev m | Fwd m | Rev m (segments, max cont.) | Initial heading corr. | In-place ° | Final pos m / yaw ° |
|---|---|---|---|---|---|---|---|---|---|
| **A** routed B→A staging | 2D, reverse allowed, no critic (prior) | SUCC | 66.1 | 0.48 | ~0.1 | 18.05 (1, 18.05) | – | 133 | 0.022 / −0.2 |
| | 2D forward-only (prior) | SUCC | 47.7 | 0.13 | 17.5 | 0 | 172° in 6.6 s | 120 | 0.066 / −0.9 |
| | **2D + PreferForward 5.0** | SUCC | **49.0** | **0.15 / 0.05** | 17.37 | **0.24 (1, 0.24)** | 143° in 6.1 s (to 0.3 m) | 47 | 0.097 / −0.6 |
| **B** free B→Home | 2D, reverse allowed (prior) | SUCC | 54.6 | – | 0 | 16.55 (1, 16.55) | ~0° | 7 | 0.047 / 1.8 |
| | 2D forward-only (prior) | SUCC | 44.6 | – | 16.64 | 0 | 178° in 9.1 s | 267 | 0.082 / 1.2 |
| | **2D + PreferForward 5.0** | SUCC | **46.3** | – | 16.21 | **0.62 (1, 0.62)** | 36° in 2.8 s (reverse-arc, see below) | 169 | 0.114 / −3.6 |
| **C** back out of pillar pocket (free) | **2D + PreferForward 5.0** | SUCC | **14.3** | – | 0 | **1.64 (1, 1.64)** | 15° | 7 | 0.083 / −1.9 |
| | forward-only control (vx_min 0), start gap ≈ 9 cm | SUCC | 21.9 | – | 1.61 | 0 | 180° in 6.5 s | 277 | 0.107 / **10.3** |
Extra runs with the same profile: routed Home→B 34.9 s, dev 0.16 m, 0 reverse; free A staging→B 24.1 s, 0 reverse. Oscillation/stall/loop: none seen in any run; near-goal dwell 6–11 s (as in earlier configs). Terminal yaw was checked against the **requested** goal, not the Nav2 result; the route-BT terminal-orientation issue (path ends at last-edge heading) does not bite here because the B and A-staging goals have yaw 0 equal to the last edge heading **[observed]**.
Figures (blue forward, red reverse, green in place): `docs/figures/rs_pf_TestA_route_B_to_Astaging.png`, `rs_pf_TestB_free_B_to_Home.png`, `rs_pf_TestC_back_out_from_pillar.png`, `rs_fwdctl_C_back_out_from_pillar.png`, and prior reverse-allowed references `rs_navfn2d_R2_route_B_to_Astaging.png`, `rs_navfn2d_A2_free_B_to_Home.png`.
Note: the logged `plan_rev_m` of NavFn plans (e.g. 16.6 m in B→Home) is an artefact — NavFn poses carry yaw 0 so any westward path reads as "reverse"; it is not a planned reverse.

## 4. Test C — geometry and what it shows
* **Verified geometry [map analysis]**: the hall is open; obstacles are isolated pillars/shelves. Everywhere in free floor the clearance exceeds the circumscribed radius 0.622 m (footprint 1.0 × 0.74), so **no location in this map strictly requires reversing** — a full in-place turn is always available away from obstacles. A reverse can only be *beneficial* (time, avoiding a spin next to an obstacle), not *necessary*. Nothing was invented beyond that.
* Scenario: the Home pillar occupies x∈[−7.63,−7.13], y∈[−0.21,0.34] (map). Setup goal (−7.38, 0.90, yaw −90°) put the robot nose-in: GT (−7.42, 0.87) → nose ≈ 2.6 cm from the pillar face, centre-to-face 0.53 m < 0.622 m, so a full spin there would sweep corners ≈ 0.1 m into the pillar. Task: go to (−7.42, 2.5), same yaw (pure straight reverse, 1.63 m). Not a real station pose; synthetic.
* Sensing/safety for backward motion [config]: one 360° 2D LiDAR (mounted 0.35 m ahead of centre, `min_height 0.15`), collision monitor `FootprintApproach` uses the footprint projected along the commanded velocity, so it is direction-agnostic, but it sees only the LiDAR plane — low obstacles or the rear overhang below the plane are not covered. **Reverse with an obstacle behind was not tested [open]**.
* **Observed**: with the critic, MPPI backed straight out (1.64 m, one segment, max command −0.277 m/s) in 14.3 s, yaw error −1.9°. The forward-only control took 21.9 s with 277° of in-place rotation and ended 10.3° off yaw (inside the 0.2 rad tolerance). Confound: the control started ≈ 6 cm further from the pillar (gap ≈ 9 cm vs 2.6 cm), where a spin was marginally possible, and n=1 each. So: the critic did **not** make reversing prohibitive at weight 5 for a 1.6 m maneuver, and the reverse was faster; but the forward-only alternative also worked, so this test shows a benefit, not a necessity.

## 5. Analysis
* **Excess reversing removed [observed]**: 18.05 m → 0.24 m (A) and 16.55 m → 0.62 m (B). Route tracking in A: 0.15 m (forward-only 0.13 m; reverse-allowed 0.48 m); duration within 1.3 s of forward-only.
* **Remaining reverse** occurs as one short segment at departure from B in both runs (red arc at the start of the figures): the robot backs out along an arc while turning before driving forward. **[inferred]** within the 2.8 s horizon a ~0.3–0.6 m back-turn costs ≲ 4.9 units, cheaper than the rest of the turn; not verified in the cost terms. In B the heading change before 0.3 m is only 36°, i.e. the turn continues forward afterwards (169° in-place total).
* **Does not discriminate by purpose**: the critic prices reverse distance in the horizon only. It cannot tell a useful back-out from a convenience reverse. In Test C the useful reverse survived at weight 5; in A/B a short convenience reverse also survived. Whether a longer genuinely useful reverse (several metres, or with a tighter budget in cost) survives was not tested.
* **No new instability seen** (n=1): no oscillation, stall or loop; final errors 0.08–0.11 m / ≤3.6° are comparable to earlier configurations.

## 6. Unintended behaviour / open
* 0.24–0.62 m reverse at B departure (small, one segment) — acceptable for a back-out, but *unrequested*; whether the 0.62 m arc keeps clearance from the rack/neighbours at B was not examined.
* Reverse with obstacles behind, repeated runs, other start poses, and the Home/A pocket with real station geometry are untested. Weight-sensitivity untested by design.
* Invalid/excluded runs (kept in `runs/`): first Nav2 launch attempt (lifecycle `change_state` response lost, auto-retried, attempt-1 log kept); `pf_TestC1_INVALID_goal_not_acknowledged`; `fwdctl_C0_INVALID_goal_not_acknowledged` (both: first goal after a pocket/launch not acknowledged by the client, no bt_navigator log line, repeated successfully). Lifecycle readiness (navigation manager + controller/bt_navigator/route_server `active`) checked before every launch's runs.

## 7. Recommendation (owner decides)
1. For this simulation, **a single reverse-capable MPPI with `PreferForwardCritic 5.0` is an acceptable provisional policy**: it removed the whole-route reversing and kept routed tracking near forward-only without new stalls, while still permitting a 1.6 m back-out (Test C).
2. It is a *soft* preference, so occasional short departure reverses will remain; if those are unacceptable at stations, or a real maneuver (dock exit, back out of a bay) must be guaranteed/limited, use an **explicitly commanded maneuver** (e.g. a bounded straight back-out behaviour or the Docking Server's undock) rather than tuning the critic; no mission-level reverse policy was implemented.
3. One justified follow-up (not run): ≥5 repeats of A/B/C plus a reverse-with-obstacle-behind test before adopting. No weight sweep was done.

## 8. Reproduction
```
scripts/launch_profile.sh <logdir> planner:=navfn structured_routing:=true mppi_path_interpretation:=2d_prefer_forward
scripts/seq_prefer_forward.sh pf          # R1 Home→B, Test A, X1 A→B, Test B
scripts/run_goal.sh pf_C0_setup_nose_in_pillar 240 "nav@-7.38@0.90@-1.5708@default" -7.38 0.90 -1.5708
scripts/run_goal.sh pf_TestC_back_out_from_pillar 150 "nav@-7.42@2.5@-1.5708@default" -7.42 2.5 -1.5708
scripts/reverse_segments.py runs/<run>/gt.csv out.json fig.png title
```
