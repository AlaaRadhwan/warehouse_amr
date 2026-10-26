#!/bin/bash
# usage: seq_lattice_forward.sh PREFIX   (sim running with planner:=lattice_forward, robot at Home)
P=$1; R="$(dirname "$0")/run_goal.sh"
$R ${P}_A1_free_Home_to_B       200 "nav@8.716@2.705@0.0@default"   8.716 2.705 0.0
$R ${P}_A2_free_B_to_Home       200 "nav@-7.589@1.2@0.0@default"   -7.589 1.2 0.0
$R ${P}_A1b_free_Home_to_B      200 "nav@8.716@2.705@0.0@default"   8.716 2.705 0.0
$R ${P}_A2b_free_B_to_Home      200 "nav@-7.589@1.2@0.0@default"   -7.589 1.2 0.0
$R ${P}_A3_free_yaw90           120 "nav@-6.5@1.2@1.5708@default"  -6.5 1.2 1.5708
# routed goal whose requested yaw (90 deg) differs from the last graph edge heading (0 deg): which yaw does the robot end with?
$R ${P}_Ryaw_route_to_B_yaw90   240 "nav@8.716@2.705@1.5708@bt"      8.716 2.705 1.5708 101,103,105
