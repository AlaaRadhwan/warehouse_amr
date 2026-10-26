#!/bin/bash
# usage: seq_routed.sh PREFIX [START_NOTE]  -- routed Home->B then B->A staging (robot must be at/near Home)
P=$1; R="$(dirname "$0")/run_goal.sh"
$R ${P}_R1_route_Home_to_B      240 "nav@8.716@2.705@0.0@bt"   8.716 2.705 0.0 101,103,105
$R ${P}_R2_route_B_to_Astaging  300 "nav@0.099@-0.175@0.0@bt"  0.099 -0.175 0.0 106,107,108
