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
