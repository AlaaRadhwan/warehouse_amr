# Structured warehouse routing — test evidence (2026-10-26)

Report: `docs/STRUCTURED_ROUTING.md`. Scripts in `scripts/`; run records in `runs/`.

* `route_compute_tests.py` — Test A (ComputeRoute by id/pose, forbidden/unknown destinations); result `runs/testA_route_compute.json`.
* `mission_run.py` — sends `GoToStation` (or direct `NavigateToPose` with the route BT via `nav@X@Y@YAW@bt`) with finite wall-time limits and cancels on timeout.
* `analyze_route_run.py` — deviation of Gazebo ground truth from the planned graph polyline, final pose error; `plan_logger.py` — logs `/plan`, `/transformed_global_plan`, `/cmd_vel_nav`.
* Runs: `testB_*` Home→B routed; `testC_home_to_A_*` Home→A staging routed; `testC1_*`, `testC2_*` B→A staging routed (departure drift, L1); `testD*` blocked/forbidden routes; `testE_*` default launch (free-space) Home→B; `testF_*` free-space B→A.
* Raw GT CSVs (`runs/*_gt.csv`, ~6 MB) and launch logs are not in Git; regenerate by repeating the runs (reproduction block in the report). Run `launch_t1..t5` = simulation launches for the runs above.
