#!/bin/bash
# usage: seq_prefer_forward.sh PREFIX  (sim running with planner:=navfn structured_routing:=true mppi_path_interpretation:=2d_prefer_forward, robot at Home)
# Home->B (routed, positions the robot at B facing east) ; Test A B->A staging (routed) ; A staging->B (free, repositions) ; Test B free B->Home
P=$1; R="$(dirname "$0")/run_goal.sh"
$R ${P}_R1_route_Home_to_B      240 "nav@8.716@2.705@0.0@bt"        8.716 2.705 0.0 101,103,105
$R ${P}_TestA_route_B_to_Astaging 300 "nav@0.099@-0.175@0.0@bt"     0.099 -0.175 0.0 106,107,108
$R ${P}_X1_free_Astaging_to_B   240 "nav@8.716@2.705@0.0@default"   8.716 2.705 0.0
$R ${P}_TestB_free_B_to_Home    240 "nav@-7.589@1.2@0.0@default"   -7.589 1.2 0.0
