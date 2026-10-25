# Disk housekeeping: proposal (nothing deleted or moved)

State at 2026-10-22: `/home` 55 GB, 50 GB used, 2.1 GB free (97%).

## What is large inside the repo
| path | size | notes |
|---|---|---|
| `calibration/nav_validation/runs/20261022_125436_after_calibration_b_to_a/bag/bag_0.mcap` | 3.2 GB | calibrated run 1; 99% is `/trajectories` (MPPI candidate rollouts, ~1.9 MB/msg) |
| other 4 `nav_validation/runs/*` | 39-50 MB each | complete bags without `/trajectories` |
| `calibration/yaw_rate_sysid/runs*` | 55 MB total | raw CSVs behind the calibration numbers |
| `debug/` | 147 MB (79 MB is one bag) | earlier failure evidence, untracked |
| `build/ install/ log/` | 35 MB | regenerable |
| `Findings/` | 0.7 MB | reports |

Everything else under `calibration/` is scripts and reports (< 1 MB).

## Large items outside the repo (your call, not touched)
`~/Projects/anymal_c_velocity` 9.5 GB, `spot_locomotion_v2` 8.9 GB, `mujoco_menagerie` 2.4 GB, `~/Downloads` 4.7 GB, `~/.cache` 3.1 GB (vscode-cpptools 1.3 GB, uv 0.7 GB, Brave 0.6 GB, pip 0.1 GB), `~/.config` 3.1 GB, `venv-ardupilot` 2.0 GB.

## Options, safest first
1. **Slim the 3.2 GB bag (recommended, frees ~3.2 GB, loses only MPPI rollouts of run 1).**
   Create a filtered copy without `/trajectories`, check it opens with `analyze_b_to_a.py` and has the same message counts for every other topic, then (with your OK) remove the original. The metrics, plots and report for that run are already written next to the bag and do not need `/trajectories`. Command shape:
   `ros2 bag convert -i bag -o bag_slim` with an output options YAML listing `exclude_topics: [/trajectories]` (needs ~40 MB free).
   If you want to keep the rollouts, alternatively move the original to external storage.
2. **Regenerable caches (~0.1-2 GB, no project data):** `~/.cache/pip` (0.1 GB), `uv cache clean` (0.7 GB), `~/.cache/vscode-cpptools` (1.3 GB, re-downloaded on demand). Brave cache 0.6 GB via the browser.
3. **`~/.ros/log` 29 MB and `~/.gz` 68 MB:** negligible.
4. **Outside the repo, only if you decide:** the big sibling projects/Downloads are not mine to judge.

## Keep
All reports, scripts, `metadata.json`/CSV/`gt.csv`/result JSON files, the four small bags, the baseline/after sysid runs (including the `*_SUPERSEDED_*` and `*_INVALID_*` runs; they are small and documented as such), `debug/`.
Do not delete `controllers.yaml` change, and none of the uncommitted work was touched.

## Needs your decision
Approve option 1 and/or 2 (or name others) and I will do exactly that and report free space before and after.
