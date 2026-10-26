# State Lattice + 2D MPPI + forward-only — evaluation (**EXPERIMENTAL**)

Continuation of `docs/NAVFN_MPPI_EVALUATION.md` on branch `experiment/navfn-mppi-evaluation`. Not adopted; defaults unchanged; route graph unchanged (temporary fixture).
Simulation, Nav2 1.3.12, single runs (n = 1–2 per condition) — **no reliability claim**. Invalid startup / client runs are excluded (listed in §6).

## 1. Added configuration
* `config/profiles/planner_lattice_forward.yaml` → `planner:=lattice_forward` (only `allow_reverse_expansion: false`; primitives, penalties, smoother unchanged).
* Existing `mppi_path_interpretation:=2d_forward_only` (`use_path_orientations false`, `PathAngle mode 0`, `enforce_path_inversion false`, `vx_min 0`). No weights/limits/tolerances/calibration changed.
* Lattice primitive set `warehouse_diff_5cm_r0.5.json` **[verified from the file]**: 16 headings (0, 26.6°, 45°, 63.4°, 90°, …), 112 primitives = **80 forward-moving + 32 in-place rotations** (one heading step each way, `rotation_penalty 2.0`) and **no reverse-moving primitives**. So reverse motion in lattice plans comes only from `allow_reverse_expansion: true`; with `false` the planner can still turn in place and drive forward (a 180° turn = ~8–9 rotation primitives).
* Plan geometry is now logged per `/plan` message (`plan_rev_m`, `plan_cusps`, `plan_inplace_poses`).

## 2. Structured B → A staging (Route Server path; connector not invoked)
`planner:=lattice mppi_path_interpretation:=2d_forward_only structured_routing:=true`, start at B facing east.

| | lattice(rev)+2D fwd-only | NavFn+2D fwd-only (previous) | baseline lattice+baseline MPPI |
|---|---|---|---|
| max / mean route deviation | **0.18 / 0.05 m** | 0.13 / 0.04 m | 1.71 / 0.80 m |
| GT forward / reverse travel | 17.6 / **0.0 m** | 17.5 / 0.0 m | ≈ 9.9 / 9.2 m |
| duration | **49.5 s** | 47.7 s | 80–81 s |
| initial heading correction | 171° in 7.2 s (before 0.3 m of travel) | 172° in 6.6 s | 14° (reversed out instead) |
| accumulated in-place rotation / total rotation | 113° / 430° | 120° / 410° | 26–30° / 427–430° |
| final position / yaw error | 0.067 m / −0.3° | 0.066 m / −0.9° | 0.045–0.083 m / −0.3…0.1° |
| terminal | 6.3 s within 0.5 m, no stall/loop | 5.5 s | 4.5 s |

* Only **one** `/plan` message (the Route Server's) was published (0 reverse, 0 cusps): the connector planner was **not invoked**, so this comparison measures the controller only; **it says nothing about State Lattice vs NavFn**. The 0.05 m / 1.8 s difference to NavFn is within what single runs can show.
* Attribution (from the previous report plus this run): the 1.71 m → ≈0.2 m improvement is due to the MPPI path interpretation; removing reversing is due to `vx_min 0`; lattice reverse expansion plays no role here.
* Figure `docs/figures/lattice2dfwd_B_to_A_staging.png`.

## 3. Free-space B → Home (default BT)
| configuration | outcome |
|---|---|
| NavFn + 2D fwd-only (previous) | 44.6 s, 0.08 m, 1.2°, 0 reverse |
| **lattice(rev) + 2D fwd-only** | **43.4 s, 0.13 m, −3.4°, 0 GT reverse**, 210° in-place; the *plan* contained up to 2.05 m of reverse and 2 cusps, which forward-only MPPI did not execute |
| **lattice(no-rev) + 2D fwd-only** (requested config) run 1 | **cancelled at 200 s wall (123 sim-s)**: stopped 0.58 m from Home, yaw −89° off, 483° in-place, MPPI commanding vx ≈ 0, |wz| ≈ 0.05 for ≈ 70 s |
| lattice(no-rev) + 2D fwd-only run 2 | SUCCEEDED but **100 s**, 462° in-place, wandered to 0.6 m from Home before converging (0.04 m, −1.9°) |
| lattice(no-rev) + **baseline** MPPI (yaw used, vx_min −0.35) | SUCCEEDED, **112.9 s**, 19.6 m travelled, 1.5 m reverse, 259° in-place, −9.5° |
Lattice(no-rev) plans were valid and forward-only (0 reverse, 0 cusps) with 14–30 in-place poses near the goal. A free yaw-90 goal at (−6.5, 1.2) after the stalled run was **cancelled at 120 s** (0.12 m, 66.7° yaw error, 313° in-place). Figure `docs/figures/lattice2dfwd_B_to_Home_free.png`.

Checks requested: valid lattice path — yes; plan without reverse — yes (`no-rev`); MPPI without reverse — yes (0 GT reverse in all `vx_min 0` runs); initial heading correction completed — yes; requested position/orientation reached — **only intermittently** in the no-reverse configuration.

## 4. Interpretation
* **Cause isolation (B → Home).** Same MPPI 2D fwd-only: reverse expansion on → fine (43 s); off → stall/dither (2 of 2). Same lattice(no-rev): 2D fwd-only → stall/100 s; baseline MPPI → 113 s. So the poor terminal behavior is associated with **disabling reverse expansion** (plan shape: forward-only arrival into the Home pocket, 0.85 m clearance next to a pillar, with in-place rotation segments), **not** with the 2D interpretation or `vx_min 0` alone. The mechanism is **unverified**: I did not inspect the costmap/critic costs; MPPI's own command was ≈ 0 while the plan was valid and aligned.
* **Does ignoring lattice yaw hurt tracking of forward-feasible paths?** Straight/arc legs: no (Home→B 35 s, ≤ 0.07 m; routed B→A 0.18 m). The path **positions** of in-place rotation segments are duplicated poses (14–30 per plan) that a yaw-blind controller cannot follow as rotations; MPPI must create the turn itself from the end-of-horizon yaw term. I could not show this is the cause of the stall because the yaw-aware baseline MPPI was also slow on the same plans.
* **Terminal orientation.** Route-Server paths end with the **last graph edge's heading**, not the requested goal yaw: routed goal (8.716, 2.705, **90°**) **SUCCEEDED while ending −90.1° from the request** (robot at yaw 0°, the e105 heading; `Ryaw_route_to_B_yaw90`). Free-space planners append the requested goal pose, so their last pose carries the requested yaw (NavFn: source; lattice: snapped to the nearest of 16 headings, max 13.3° error). A routed station arrival is therefore correct only if the last edge points along the station yaw, or a last-mile/orientation step is added.
* Where the connector is not invoked (routes on nodes) planner choice is irrelevant; this report does not conclude that lattice is better or worse than NavFn from those runs.

## 5. Findings summary
* Lattice(rev) + 2D MPPI + forward-only is compatible with the Route Server path and with ordinary free-space goals in these runs, with plans that still contain reverse segments (not executed).
* Lattice(no-rev) is **not** acceptable as configured: reproducible terminal stall/dithering near Home.
* Unexpected: 462–483° of in-place rotation in the no-reverse B → Home runs; forward-only B → Home needs an initial 180° turn (≈ 150–270° in-place in all forward-only runs).
* **Recommendation:** retain **State Lattice with reverse expansion enabled** as the free-space/connector planner for further development, paired with 2D MPPI settings and the owner's reverse-policy decision (`vx_min 0` or not): it matched NavFn within noise in both comparable scenarios (B→Home 43 vs 45 s; routed B→A 49.5 vs 47.7 s) and keeps kinematic, footprint-aware planning. **Do not** use `allow_reverse_expansion: false`. The evidence is thin (one run each): NavFn remains an equally plausible simple alternative; the open risk for lattice(rev) is a mismatch between plans that contain reverse/cusps and a controller that never reverses (untested: tracking deviation from the plan near cusps; goals that truly require backing up).

## 6. Full run table and exclusions
| run | result | dur s | travelled m | final pos err m | final yaw err ° | GT reverse m | in-place rot ° | max dev from route m |
|---|---|---|---|---|---|---|---|---|
| lattice(rev)+baseline MPPI, free Home→B | – | 35.3 | 16.53 | 0.054 | 1.0 | 0.0 | 0 | – |
| lattice(rev)+baseline MPPI, free B→A staging | – | 48.5 | 9.72 | 0.105 | -2.6 | 1.45 | 102 | – |
| lattice(rev)+baseline MPPI, routed Home→B | – | 34.5 | 16.77 | 0.041 | 1.1 | 0.0 | 0 | 0.16 |
| lattice(rev)+baseline MPPI, routed B→A staging (run 1) | – | 79.7 | 18.99 | 0.045 | -0.3 | 9.14 | 30 | 1.71 |
| lattice(rev)+baseline MPPI, routed B→A staging (run 2) | – | 81.1 | 19.17 | 0.083 | 0.1 | 9.24 | 26 | 1.71 |
| NavFn+2D MPPI (vx_min −0.35), A1_free_Home_to_B | SUCCEEDED | 34.7 | 16.54 | 0.062 | 0.3 | 0.0 | 0 | – |
| NavFn+2D MPPI (vx_min −0.35), A2_free_B_to_Home | SUCCEEDED | 54.6 | 16.56 | 0.047 | 1.8 | 16.55 | 7 | – |
| NavFn+2D MPPI (vx_min −0.35), A3_free_yaw90 | SUCCEEDED | 12.7 | 1.28 | 0.084 | 1.0 | 0.14 | 33 | – |
| NavFn+2D MPPI (vx_min −0.35), R1_route_Home_to_B | SUCCEEDED | 35.4 | 16.77 | 0.012 | 0.5 | 0.0 | 0 | 0.15 |
| NavFn+2D MPPI (vx_min −0.35), R2_route_B_to_Astaging | SUCCEEDED | 66.1 | 18.15 | 0.022 | -0.2 | 18.05 | 133 | 0.48 |
| lattice(rev)+2D MPPI (vx_min −0.35), R1_route_Home_to_B | SUCCEEDED | 35.1 | 16.82 | 0.071 | 0.2 | 0.0 | 0 | 0.16 |
| lattice(rev)+2D MPPI (vx_min −0.35), R2_route_B_to_Astaging | SUCCEEDED | 65.4 | 18.09 | 0.029 | 1.7 | 17.98 | 104 | 0.41 |
| NavFn+2D MPPI forward-only, A1_free_Home_to_B | SUCCEEDED | 34.4 | 16.51 | 0.037 | 0.7 | 0.0 | 0 | – |
| NavFn+2D MPPI forward-only, A2_free_B_to_Home | SUCCEEDED | 44.6 | 16.72 | 0.082 | 1.2 | 0.0 | 267 | – |
| NavFn+2D MPPI forward-only, A3_free_yaw90 | SUCCEEDED | 12.5 | 1.12 | 0.019 | 4.3 | 0.0 | 66 | – |
| NavFn+2D MPPI forward-only, R1_route_Home_to_B | SUCCEEDED | 34.6 | 15.9 | 0.017 | -0.5 | 0.0 | 0 | 0.63 |
| NavFn+2D MPPI forward-only, R2_route_B_to_Astaging | SUCCEEDED | 47.7 | 17.51 | 0.066 | -0.9 | 0.0 | 120 | 0.13 |
| lattice(rev)+2D MPPI forward-only, R1_route_Home_to_B | SUCCEEDED | 35.6 | 16.79 | 0.027 | 0.7 | 0.0 | 0 | 0.16 |
| lattice(rev)+2D MPPI forward-only, R2_route_B_to_Astaging | SUCCEEDED | 49.5 | 17.6 | 0.067 | -0.3 | 0.0 | 113 | 0.18 |
| lattice(rev)+2D MPPI forward-only, A1_free_Home_to_B | SUCCEEDED | 35.3 | 16.54 | 0.066 | 0.4 | 0.0 | 0 | – |
| lattice(rev)+2D MPPI forward-only, A2_free_B_to_Home | SUCCEEDED | 43.4 | 16.83 | 0.132 | -3.4 | 0.0 | 210 | – |
| lattice(no-rev)+2D MPPI forward-only, A1_free_Home_to_B | SUCCEEDED | 34.9 | 16.51 | 0.042 | 2.3 | 0.0 | 1 | – |
| lattice(no-rev)+2D MPPI forward-only, A2b_free_B_to_Home | TIMEOUT-cancelled | 122.8 | 17.14 | 0.58 | -88.7 | 0.0 | 483 | – |
| lattice(no-rev)+2D MPPI forward-only, A2c_free_B_to_Home | SUCCEEDED | 100.0 | 18.01 | 0.042 | -1.9 | 0.0 | 462 | – |
| lattice(no-rev)+2D MPPI forward-only, A3_free_yaw90 | TIMEOUT-cancelled | 77.3 | 2.27 | 0.118 | 66.7 | 0.0 | 313 | – |
| lattice(no-rev)+2D MPPI forward-only, Ryaw_route_to_B_yaw90 | SUCCEEDED | 36.7 | 16.04 | 0.024 | -90.1 | 0.0 | 35 | 0.52 |
| lattice(no-rev)+baseline MPPI, A1_free_Home_to_B | SUCCEEDED | 35.5 | 16.48 | 0.033 | 4.3 | 0.0 | 1 | – |
| lattice(no-rev)+baseline MPPI, A2_free_B_to_Home | SUCCEEDED | 112.9 | 19.57 | 0.085 | -9.5 | 1.5 | 259 | – |

Excluded as invalid (kept in `runs/`): `latfwd_A2_INVALID_goal_not_acknowledged` (client did not receive the goal acknowledgement; the goal never started), `latfwd_A1b_INVALID_robot_already_at_goal` (consequence). Startup: every launch in this report passed `launch_profile.sh` readiness checks (navigation lifecycle manager active, controller/bt_navigator/route_server `active`).
`result.json sim_duration_s` is unreliable; use `metrics.json duration_s`.

## 7. Reproduction
```
bash investigations/navfn_mppi_evaluation_2026-10-26/scripts/launch_profile.sh LOGDIR structured_routing:=true planner:=lattice mppi_path_interpretation:=2d_forward_only
investigations/navfn_mppi_evaluation_2026-10-26/scripts/seq_routed.sh PREFIX                # Home->B, B->A staging (routed)
investigations/navfn_mppi_evaluation_2026-10-26/scripts/seq_ab.sh PREFIX                    # free Home->B, B->Home
# planner:=lattice_forward ...   investigations/.../scripts/seq_lattice_forward.sh PREFIX    # incl. routed goal with yaw 90 deg
python3 investigations/navfn_mppi_evaluation_2026-10-26/scripts/recompute_all.py; python3 investigations/navfn_mppi_evaluation_2026-10-26/scripts/initial_rotation.py runs/<run>/gt.csv runs/<run>/plog.jsonl
```
