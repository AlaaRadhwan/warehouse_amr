# Data manifest — locally retained, not versioned in Git

Generated 2026-10-25 for the checkpoint on branch `feature/bidirectional-navigation-baseline`.
All paths are relative to the repository root. These files exist **only on the original workstation**
(`~/Projects/warehouse_amr`). Git contains the scripts, reports, figures and small per-run records, not these files.

> **Backup warning.** The rosbags below are the primary raw evidence for every navigation and calibration
> investigation and **cannot be regenerated exactly** (Gazebo runs are not bit-reproducible; wall-clock load and
> scheduling differ). They are not backed up by Git. Copy `*.mcap` to an independent location
> (external disk / NAS / cloud storage) and verify against the SHA-256 values below
> (`sha256sum -c` format: `<hash>  <path>`). Treat the hashes as the identity of each recording.

## Summary

| Category | Files | Total size | Regenerable? |
|---|---:|---:|---|
| Rosbag2 `.mcap` recordings | 21 | 688.7 MB | **No** — primary evidence |
| `extracted.pkl.gz` / `series.pkl` | 12 | 31.3 MB | Yes, from the bags |
| per-run `metrics.json` | 10 | 16.4 MB | Yes, from the pickles |
| `*_SUPERSEDED_*` scratch calibration runs | 21 | 14.5 MB | No (superseded; kept for the record) |
| Duplicate `.zip` archives | 3 | 4.9 MB | Duplicates of committed directories |
| `action_client.log` | 1 | 8.7 MB | No (verbose feedback log) |

The small `metadata.yaml` of every bag **is** committed next to where the bag lives (`.../bag/metadata.yaml`),
so topics, durations and message counts are documented even without the `.mcap`.

## Regenerating derived outputs

Prerequisites: ROS 2 Jazzy (`rosbag2_py`), Python 3 with numpy/pandas/matplotlib/PyYAML, and the original bags in place.

```bash
source /opt/ros/jazzy/setup.bash
cd investigations/btoa_variability_2026-10-23
for run in runs/*/; do
  [ -d "$run/bag" ] || continue
  python3 scripts/classify.py  "$run"     # -> classification.json (from gt.csv + client result; no bag needed)
  python3 scripts/extract.py   "$run"     # bag -> extracted.pkl.gz
  python3 scripts/analyze_run.py "$run"   # -> analysis/<run>/metrics.json + per-run figures
done
python3 scripts/compare_runs.py           # -> analysis/summary_table.{csv,md}, analysis/cmp_*.png
python3 scripts/addendum_compare.py       # -> analysis/addendum/series.pkl + aligned_comparison.txt etc.
python3 scripts/addendum_figures.py       # addendum figures (needs series.pkl)
```

Notes and limits:
* The committed `summary_table.*`, `terminal_entry_state.txt`, figures and addendum text are the outputs that were
  used in the reports. `classification.json` can be recomputed from the committed `gt.csv`/client results alone;
  the metrics, comparison tables and addendum need the bags (via `extracted.pkl.gz`).
* Pickle files are unsafe to load from untrusted sources; load only pickles you generated yourself.
* Calibration and earlier validation analyses (`calibration/yaw_rate_sysid/`, `calibration/nav_validation/analyze_*.py`,
  `replay_watchdog.py`, `debug/*.py`) read the bags in the corresponding run directories; see each directory's report.
* Some scripts contain absolute paths from the original machine; adjust them if the repository is checked out elsewhere.
* Regeneration reproduces the *analysis*, not the *experiments*; new simulations will give different trajectories.

## Rosbag recordings (`*.mcap`)

| Path | Size (bytes) | SHA-256 |
|---|---:|---|
| `calibration/nav_validation/runs/20261022_130027_INVALID_control_runtime_param_set_not_applied/bag/bag_0.mcap` | 38576128 | `c692808375970e6e60656d7658bda2b30d67e4a5f048763d208b4582be5773d1` |
| `calibration/nav_validation/runs/20261022_130437_after_calibration_repeat2/bag/bag_0.mcap` | 38395904 | `3b5ce72afeeb72482b84fb478e50550bffec55e75dd601f50627a10718e75585` |
| `calibration/nav_validation/runs/20261022_130824_after_calibration_repeat3/bag/bag_0.mcap` | 40235008 | `3da5bab02dac075573c53cfdbd9356bc869ec0f8ae8eddd136244a0e0de7883d` |
| `calibration/nav_validation/runs/20261022_131314_control_uncalibrated_multiplier_1.0_scratch_controllers/bag/bag_0.mcap` | 50774016 | `bd2e43ea28918bc97c3d5b73d673c2aeb72e55a92b68da1ff4d4ebc5811e62ed` |
| `debug/nav_problem_b_to_a/nav_problem_b_to_a_0.mcap` | 18136891 | `9e5a683df7f409c9d523055560952d93c1d784c8e518732172b6a57065f5481b` |
| `debug/nav_problem_home_to_b/nav_problem_home_to_b_0.mcap` | 11930589 | `8225b2514e0947ee037b44929bf7bdd8edce45571f09a3c7c1b6788ffdbb9c7b` |
| `debug/nav_test_b_to_a_bidirectional/bag/bag_0.mcap` | 73011558 | `893b31c71b5a760a22c990bfac58b824d63d53ca67b7942269951d1f447941d2` |
| `debug/nav_test_b_to_a_yaw_pi/nav_test_b_to_a_yaw_pi_0.mcap` | 24931132 | `e767e7cd6dd4306969edfdb562bcc464ecac590f6fe6999333c082936d9e11d8` |
| `debug/nav_test_b_to_a_yaw_pi_pathFollowCritic_thresh_0.10/nav_test_b_to_a_yaw_pi_pathFollowCritic_thresh_0.10_0.mcap` | 16152282 | `3a40b79a54a267b620212b62f1dbf895d94509b829b75ca8cb6a43363ebbe92e` |
| `investigations/btoa_variability_2026-10-23/runs/e01_m1.0/bag/bag_0.mcap` | 32961376 | `376dcf5e2b5dbe98f373646262430b3de888ff8e46990d3c1661324bd1523585` |
| `investigations/btoa_variability_2026-10-23/runs/e02_m0.92754/bag/bag_0.mcap` | 39058568 | `d333004899d9b2c6010c37a6df71dc6a0222514e8c76a0e950ab69647507c042` |
| `investigations/btoa_variability_2026-10-23/runs/e03_m1.0_retry1/bag/bag_0.mcap` | 28233172 | `97da3446f5e946fb44c45645507290688de86fad790ec318477b3a8c4d270ae4` |
| `investigations/btoa_variability_2026-10-23/runs/e04_m0.92754/bag/bag_0.mcap` | 31316643 | `59dbdacf0ef1c6426eb28da6bdd3574390d8b6686b3b8f4b99c9ced1e063ea45` |
| `investigations/btoa_variability_2026-10-23/runs/e05_m1.0/bag/bag_0.mcap` | 28725402 | `d5119ec2d036338c0acaeb3b1575d56e42d245282e2e6844d4fe55b4b09f2a17` |
| `investigations/btoa_variability_2026-10-23/runs/p01_m1.0/bag/bag_0.mcap` | 32785501 | `ff2d545082f31d5cf1494090406e556bbe4dc147a3fb5b260701ef1f3b47452d` |
| `investigations/btoa_variability_2026-10-23/runs/p02_m0.92754/bag/bag_0.mcap` | 33302994 | `9f2242ef8a715979c3d2698d7fe4d39cdec521e1f9f23ea7d3652062089ccf09` |
| `investigations/btoa_variability_2026-10-23/runs/p03_m1.0/bag/bag_0.mcap` | 28916353 | `d45f6fe32ff2c85c9f6c084857ac3bc00bdf12e5996282734d729b3bbe25184e` |
| `investigations/btoa_variability_2026-10-23/runs/p04_m0.92754_retry1/bag/bag_0.mcap` | 39843234 | `5318d45f41e7869948443b819ccae57eb4ed5f164f24ad1f0eda2134ddd78dd4` |
| `investigations/btoa_variability_2026-10-23/runs/p06_m0.92754/bag/bag_0.mcap` | 38451674 | `cbef5a429c1a2b2a086a7e078216487602a238552c0c743974598015d8097bd0` |
| `investigations/btoa_variability_2026-10-23/runs/smoke01_m0.92754/bag/bag_0.mcap` | 22593536 | `4c0e44977f99d3f4e5007ee590b83252e592d39f8c11ca092871a21c7cf355de` |
| `investigations/btoa_variability_2026-10-23/runs/smoke02_m1.0/bag/bag_0.mcap` | 20365278 | `886754c9c4901b4a98b9053e7713f3c1cb5af2c9107547c984a077843875c11e` |

## Derived pickles

| Path | Size (bytes) | SHA-256 |
|---|---:|---|
| `investigations/btoa_variability_2026-10-23/analysis/addendum/series.pkl` | 2841488 | `c1fd93859d09589cfe63dd355aac71d2e404b59739d128f6d36c06af79e0d129` |
| `investigations/btoa_variability_2026-10-23/runs/e01_m1.0/extracted.pkl.gz` | 2734973 | `830188281e45d8d8acd4ea976e578308ae0b2bb6b9b6c60cb03f0ffa5d50df87` |
| `investigations/btoa_variability_2026-10-23/runs/e02_m0.92754/extracted.pkl.gz` | 3070623 | `3e602bb082bf510a2ebd8d5d6cbed64e37f72242305bd32a43112294bb1e5806` |
| `investigations/btoa_variability_2026-10-23/runs/e03_m1.0_retry1/extracted.pkl.gz` | 2382249 | `e26b727e3373164b1c05e2318ff0a0d5436c88c9b4743bddeecbe9389d129abf` |
| `investigations/btoa_variability_2026-10-23/runs/e04_m0.92754/extracted.pkl.gz` | 2567028 | `b65eeed4bf1ef4ee875812753e50c246393b46c55b1355fc83a95561c725a026` |
| `investigations/btoa_variability_2026-10-23/runs/e05_m1.0/extracted.pkl.gz` | 2350877 | `ec276dd1171ee7419d20bc46ba61b053bd7933b8e96b8631c6a138ff42f16f17` |
| `investigations/btoa_variability_2026-10-23/runs/p01_m1.0/extracted.pkl.gz` | 2723756 | `96c568f0c705a964292b8085f85f9a23d25081f0e61b2fa6d91ef2333ffd3db9` |
| `investigations/btoa_variability_2026-10-23/runs/p02_m0.92754/extracted.pkl.gz` | 2541096 | `89979670158bbe91ec3aadff18164b2e698afbde8a3e47dd74cfc342cf4719bb` |
| `investigations/btoa_variability_2026-10-23/runs/p03_m1.0/extracted.pkl.gz` | 2222834 | `3a89dacc6683631141d514eeea63e6298993715fc754c1d3e4c54eca228ba06e` |
| `investigations/btoa_variability_2026-10-23/runs/p04_m0.92754_retry1/extracted.pkl.gz` | 3143531 | `64bb5477cda0a313f341d52be882803091522a2f76ddd165bca7acea8c4ccc54` |
| `investigations/btoa_variability_2026-10-23/runs/p06_m0.92754/extracted.pkl.gz` | 2981281 | `a7ebe5ea114e75f888c5ca21ab360ba3cc7f256bdd4d4d7389ad4a21757c56da` |
| `investigations/btoa_variability_2026-10-23/runs/smoke01_m0.92754/extracted.pkl.gz` | 1785615 | `d6580767f25baab2d2e52b3ee39ccf015f29648062db717705012e8623efeb6d` |

## Derived metrics

| Path | Size (bytes) | SHA-256 |
|---|---:|---|
| `investigations/btoa_variability_2026-10-23/analysis/e01_m1.0/metrics.json` | 1700817 | `433cd97fa67bf5cd555772ca8453781232eba338ca10d943fc115bc6248b8814` |
| `investigations/btoa_variability_2026-10-23/analysis/e02_m0.92754/metrics.json` | 1937139 | `9da6e96f6ca2e49af3b8870000fa3be16c402ed386ee00733c2a66316a21a653` |
| `investigations/btoa_variability_2026-10-23/analysis/e03_m1.0_retry1/metrics.json` | 1377066 | `2de5a1a21d16f48f2bc4713f15fa5f1480ca6f48865c8e633526826f7c61301b` |
| `investigations/btoa_variability_2026-10-23/analysis/e04_m0.92754/metrics.json` | 1559210 | `cf268c3cb25cbbc9151d4611256ded7c326a34eada9fc24249a5589e9c4e1d42` |
| `investigations/btoa_variability_2026-10-23/analysis/e05_m1.0/metrics.json` | 1390219 | `55e268220b45390edc050aff533f5bf7088d3dedcc66f6b7b0ba54c21afd87f8` |
| `investigations/btoa_variability_2026-10-23/analysis/p01_m1.0/metrics.json` | 1727142 | `3b7db17285f671651aedb41083df3546af24349801126562923666ecde64ccaa` |
| `investigations/btoa_variability_2026-10-23/analysis/p02_m0.92754/metrics.json` | 1538751 | `150adb86baf43772a881b4688f05bc798d1f2f02933477ce0c31cf4bf93c9cad` |
| `investigations/btoa_variability_2026-10-23/analysis/p03_m1.0/metrics.json` | 1283464 | `65768eabb3d40e17950ce80033871dd35c4198f0111154b6822ea39cf019d2b7` |
| `investigations/btoa_variability_2026-10-23/analysis/p04_m0.92754_retry1/metrics.json` | 2001888 | `dc1ce99f19350148147a9a2be58e9579e6067ad8f68afaf11cd70a20347e14dc` |
| `investigations/btoa_variability_2026-10-23/analysis/p06_m0.92754/metrics.json` | 1858075 | `37a25af9cddb7b1c84266cd19fd7b318421da9c7c9b611289abf412b5e2d6a99` |

## Superseded scratch runs (preserved locally)

These were recorded while orphaned `gz sim` servers shared the Gazebo transport (mixed data) and were replaced by clean
single-simulation reruns (see `calibration/yaw_rate_sysid/` reports).

| Path | Size (bytes) | SHA-256 |
|---|---:|---|
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/cmd.csv` | 112227 | `a4e0d6c04dc494fc3d88bea4dcc77d9900b7ab0ec9b5e887d983befd23fb554d` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/cmd_out.csv` | 267469 | `5b9aa1f7d410edf50f750a93b82bc2187028f0cfd5f9bda9af23f0cf8ea628a0` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/gt.csv` | 1291702 | `ce09628d209022ced125e797a4cf758f7c638ae31be986ed80eef7314f9c9ab7` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/imu.csv` | 1858710 | `de1aa1c066f0a0480b128abe57c27be869f638555830dd8f7832f480cad44fd8` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/joints.csv` | 794032 | `f37275bd1fab7f7a458dfe9b2a4b6ebc54215f19af9bc284d3c3ffa7d035b194` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/linear_ratios.png` | 73614 | `434cb2bb3a4e09348e5fd14fa406e34803363a51d7fad183e607d200d581769f` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/linear_report.md` | 2386 | `f7b2ca23ce43ae4012a79bd09192c928df606fa924a6c1c9490e5fcd284a6c6c` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/linear_summary.csv` | 3302 | `3f22b1b15e4c499deee1208a2ebe3cbdd7716d1f458b58248d64e3f22e369d09` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/metadata.json` | 3297 | `8c138fb86d9d706b21520711bc67db14c04a9d1fa7822c41b21f39350b7df614` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/odom_fused.csv` | 544839 | `7ca04d63d2075ad3f59d598d3d32b80f613fa23683fc3e5a7880135bc65f000b` |
| `calibration/yaw_rate_sysid/runs/20261022_122400_linear_SUPERSEDED_multisim/odom_raw.csv` | 1218784 | `45ee54d6231f78c4d6d2c479342475fac749b3efb386d29f7eaf0c3f8b428334` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/cmd.csv` | 65371 | `57867e31edd25103696f7ccf4b2471239ec1c661c027d7c5c4f177a06749e5a4` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/cmd_out.csv` | 165709 | `d8c10c632b15344fcf4cd177eb434e6a399f828628668b92727b356cc84100a9` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/gt.csv` | 2044849 | `630f93791b61cb9431b61ece42c71f614f1cf02fae14f237f85af1661a834e3b` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/imu.csv` | 3914937 | `a5845a4d30b37fe2809b03d51a94274b9c8e2753e1ba78f118a4b7a0e8e24bdf` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/joints.csv` | 510497 | `3be9a218f79025943e1d03b320ed1e33c9cea5b5858afcdc3f522a2a7a11596a` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/metadata.json` | 2674 | `ec3c8e9d6335ed89c7394e228e4d4dcf9b6399e8bbf3dc2800a96abed9b0faa6` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/odom_fused.csv` | 774723 | `aff7c3804b5023241d9f2fcbe57b4f26d351fe5f7d2c964dfbf354a9b99f8b98` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/odom_raw.csv` | 821121 | `ed5d4e6471303cc397c5c25f5fe0c14ff248e9ea2e1776065562213dea8a0f12` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/report.md` | 2402 | `713e441fd5d7dc3829f82b29cf7116dc7a6095a17358e5400912cc886da8eb17` |
| `calibration/yaw_rate_sysid/runs_diagnostic/20261022_122848_rot_wheelwidth_0.10_SUPERSEDED_multisim/yaw_rate_summary.csv` | 2471 | `77838871f51924abb93333c06de20c8e8b7eed2a8e1817e7298879707a6d3031` |

## Duplicate archives and logs

| Path | Size (bytes) | SHA-256 |
|---|---:|---|
| `Findings/Warehouse_AMR_Investigation_Report_and_Figures.zip` | 507726 | `8f434584bdd2129c3c9c51f362ce8fa3b7b8f6ca5724dd1ae75f34865f7551e0` |
| `calibration/yaw_rate_sysid/runs/20261022_102121_baseline.zip` | 2811296 | `6e23466bdfc0eb7fd188057005da9127dc8aa355b353224bfcd5491178606559` |
| `calibration/yaw_rate_sysid/runs/20261022_123234_linear.zip` | 1598086 | `e242dca0f7bdb8abca25b2f40e592b8dd7d936c430d1f24c6b4ffefd0ac66c84` |
| `debug/nav_test_b_to_a_bidirectional/action_client.log` | 8744555 | `6bbca3d8584b4daf7f9f93f14e635d0fde5280d1c41efb947422d5870ffcb391` |

