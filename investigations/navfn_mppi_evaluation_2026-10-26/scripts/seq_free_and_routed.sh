#!/bin/bash
# usage: seq_free_and_routed.sh PREFIX  (sim must already be running with the profile under test, robot at Home)
P=$1; R="$(dirname "$0")/run_goal.sh"
$R ${P}_A1_free_Home_to_B       200 "nav@8.716@2.705@0.0@default"   8.716 2.705 0.0
$R ${P}_A2_free_B_to_Home       200 "nav@-7.589@1.2@0.0@default"   -7.589 1.2 0.0
$R ${P}_A3_free_yaw90           120 "nav@-6.5@1.2@1.5708@default"  -6.5 1.2 1.5708
$R ${P}_R1_route_Home_to_B      240 "nav@8.716@2.705@0.0@bt"        8.716 2.705 0.0 101,103,105
$R ${P}_R2_route_B_to_Astaging  300 "nav@0.099@-0.175@0.0@bt"      0.099 -0.175 0.0 106,107,108
