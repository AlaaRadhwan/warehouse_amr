# Limitations and future work

**Scope.** Simulation only (Gazebo, real-time factor about 0.6). Nothing has run on hardware. The wheel-separation calibration
(0.92754) is a simulator calibration. Nav2's Collision Monitor is not a substitute for independent physical safety measures.

## What the evidence supports
* The accepted baseline (NavFn + 2D-path MPPI + `PreferForwardCritic` + Route Server) completed the four tests in
  [`CONSOLIDATED_VALIDATION.md`](CONSOLIDATED_VALIDATION.md) in simulation, with the requested goal yaw honored after the routed-yaw fix.
* This is **functional acceptance from single runs (n = 1 per test)**. It is not statistical reliability, not a safety case, and not
  deployment readiness.

## Known limitations and open issues
| Area | Status |
|---|---|
| Route graph | Provisional test fixture (6 nodes, 8 edges, no outbound edge from Station A staging, no zones or speed limits). Structured routing is opt-in. |
| Reverse motion | A soft preference; short back-outs (0.1 to 0.7 m observed). It cannot tell useful from convenient reversing. A bounded undock maneuver would be the explicit alternative. |
| Obstacles | Only a runtime-spawned box in the LiDAR plane was tested; low obstacles below the plane are not covered. A 0.05 m footprint margin occurred at a goal near the box; the minimum goal margin is undecided. |
| Goal yaw | A deliberately unreachable yaw (should fail cleanly) was not tested. |
| Collision Monitor | Velocity intervention could not be confirmed at the 10 Hz sampling used. |
| Docking | `DockRobot` call path and AprilTag detection exist in source; end-to-end docking on the accepted baseline is not re-validated. `amr_perception/launch/apriltag.launch.py` is not part of the one-command bringup. Docking and undocking are not integrated into the route behavior tree. |
| Startup | Nav2 lifecycle startup occasionally hung on a lost response (about 2 of 9 launches); the first goal after launch is sometimes not acknowledged by the test client. |
| Diagnostics | MPPI `visualize: true` in `nav2_params.yaml` is a diagnostic setting that has not been reverted. |
| Launch defaults | Defaults still start the earlier Smac lattice + MPPI stack; the baseline is opt-in via launch arguments. |
| Earlier B→A spin/stall | 1 of 5 runs spun with the uncalibrated multiplier, 0 of 5 with 0.92754 (small sample). One stall with the calibrated multiplier was not reproduced and is unexplained. |
| Rotation Shim | Evaluated on a separate experimental branch (`feature/rotation-shim-integration`); unfinished and not part of the baseline. |
| Data | Raw rosbags are not in Git ([`DATA_MANIFEST.md`](DATA_MANIFEST.md)); some figures cannot be regenerated from a clone. |
| Automated tests | No CI. Only the calibration watchdog has a unit test; the lint tests are the unmodified ROS 2 templates. |

## Planned work (not started unless stated)
1. Owner-designed warehouse route network replacing the provisional graph.
2. Station-aware navigation, then docking/undocking integrated with routing, then complete mission execution.
3. A goal-margin policy and tests for low obstacles and unreachable goals.
4. Repeat runs to obtain variability estimates before making any reliability statement.
5. Hardware plan: re-identify kinematics and re-validate safety on the real platform.
