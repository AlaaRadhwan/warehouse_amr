#!/bin/bash
# usage: seq_ab.sh PREFIX   (robot at Home) : free Home->B then free B->Home
P=$1; R="$(dirname "$0")/run_goal.sh"
$R ${P}_A1_free_Home_to_B 200 "nav@8.716@2.705@0.0@default" 8.716 2.705 0.0
sleep 5
$R ${P}_A2_free_B_to_Home 200 "nav@-7.589@1.2@0.0@default" -7.589 1.2 0.0
