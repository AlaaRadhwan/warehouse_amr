| run | dur s | travelled m | final pos err m | final yaw err ° | GT reverse m | in-place rot ° | max dev from route m |
|---|---|---|---|---|---|---|---|
| lattice+baseline MPPI, free Home→B | 35.3 | 16.53 | 0.054 | 1.0 | 0.0 | 0 | – |
| lattice+baseline MPPI, free B→A staging | 48.5 | 9.72 | 0.105 | -2.6 | 1.45 | 102 | – |
| lattice+baseline MPPI, routed Home→B | 34.5 | 16.77 | 0.041 | 1.1 | 0.0 | 0 | 0.16 |
| lattice+baseline MPPI, routed B→A staging (run 1) | 79.7 | 18.99 | 0.045 | -0.3 | 9.14 | 30 | 1.71 |
| lattice+baseline MPPI, routed B→A staging (run 2) | 81.1 | 19.17 | 0.083 | 0.1 | 9.24 | 26 | 1.71 |
| NavFn+2D MPPI, A1_free_Home_to_B | 34.7 | 16.54 | 0.062 | 0.3 | 0.0 | 0 | – |
| NavFn+2D MPPI, A2_free_B_to_Home | 54.6 | 16.56 | 0.047 | 1.8 | 16.55 | 7 | – |
| NavFn+2D MPPI, A3_free_yaw90 | 12.7 | 1.28 | 0.084 | 1.0 | 0.14 | 33 | – |
| NavFn+2D MPPI, R1_route_Home_to_B | 35.4 | 16.77 | 0.012 | 0.5 | 0.0 | 0 | 0.15 |
| NavFn+2D MPPI, R2_route_B_to_Astaging | 66.1 | 18.15 | 0.022 | -0.2 | 18.05 | 133 | 0.48 |
| lattice+2D MPPI, R1_route_Home_to_B | 35.1 | 16.82 | 0.071 | 0.2 | 0.0 | 0 | 0.16 |
| lattice+2D MPPI, R2_route_B_to_Astaging | 65.4 | 18.09 | 0.029 | 1.7 | 17.98 | 104 | 0.41 |
| NavFn+2D MPPI forward-only, A1_free_Home_to_B | 34.4 | 16.51 | 0.037 | 0.7 | 0.0 | 0 | – |
| NavFn+2D MPPI forward-only, A2_free_B_to_Home | 44.6 | 16.72 | 0.082 | 1.2 | 0.0 | 267 | – |
| NavFn+2D MPPI forward-only, A3_free_yaw90 | 12.5 | 1.12 | 0.019 | 4.3 | 0.0 | 66 | – |
| NavFn+2D MPPI forward-only, R1_route_Home_to_B | 34.6 | 15.9 | 0.017 | -0.5 | 0.0 | 0 | 0.63 |
| NavFn+2D MPPI forward-only, R2_route_B_to_Astaging | 47.7 | 17.51 | 0.066 | -0.9 | 0.0 | 120 | 0.13 |
