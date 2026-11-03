# Third-party notices

Summary of material this project is derived from or refers to. Evidence, status and open questions are in
[`THIRD_PARTY_REVIEW.md`](THIRD_PARTY_REVIEW.md). This is not legal advice and does not grant rights in any third-party work.

## Derived material in this repository

| Material | Source | Licence |
|---|---|---|
| `src/amr_simulation/worlds/warehouse_world.sdf` | Adapted from `clearpath_gz/worlds/warehouse.sdf`, [clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator) (`jazzy`) | BSD-3-Clause, Copyright (c) 2023 clearpathrobotics. Notice: [`NOTICE-clearpath.md`](src/amr_simulation/worlds/NOTICE-clearpath.md) |
| `src/amr_navigation/config/lattice/warehouse_diff_5cm_r0.5.json` | Unmodified sample from `nav2_smac_planner` | Apache-2.0 |
| `src/amr_navigation/behavior_trees/navigate_on_warehouse_route.xml` | Adapted from a Nav2 behavior tree | Apache-2.0 |
| `investigations/btoa_variability_2026-10-23/analysis/addendum/mppi_1.3.12_source_used/` | 19 unmodified `nav2_mppi_controller` 1.3.12 files, citation only | Apache-2.0 headers (see the notice in that folder) |

## Referenced, not included: Gazebo Fuel models

Downloaded by Gazebo at run time from <https://fuel.gazebosim.org>; not stored in this repository. They are separate works under
their own licences, **not** covered by the Clearpath BSD-3 notice. Licences read from the Fuel API on 2026-11-03.

| Model | Licence | Author (cached model.config) |
|---|---|---|
| OpenRobotics/Warehouse v5 | CC BY-NC-ND 4.0 | Filipe Almeida |
| MovAi/shelf_big v2 | CC BY-NC-ND 4.0 | Filipe Almeida |
| MovAi/shelf v1 | CC BY-NC 4.0 | Filipe Almeida |
| OpenRobotics/Chair v3 | CC BY 4.0 | Cole Biesemeyer |
| OpenRobotics/foldable_chair v3 | CC BY 4.0 | Rumman Waqar |
| OpenRobotics/MaleVisitorOnPhone v2 | CC BY 4.0 | Wan Yi Seow |
| OpenRobotics/Jersey Barrier v6, Table v4, Rescue Randy Sitting v8 (inactive), plateau/Casual female v1 | CC0 1.0 | n/a |

Images, maps and videos derived from or showing these models remain subject to those licences; check the licence of each model before reuse.

## Installed dependencies (not redistributed)

Nav2 and most ROS 2 packages: Apache-2.0; `nav2_mppi_controller`, `apriltag_ros`, `apriltag_msgs`: MIT; `slam_toolbox`: LGPL;
`nav2_amcl`: LGPL-2.1-or-later.
