#!/bin/bash
# Consolidated validation: ONE launch of the recommended profile, 4 primary tests (T1..T4) + setup moves (S*, not tests).
# usage: run_validation.sh   (from anywhere; writes runs/<name>/ and runs/validation.out)
cd "$(dirname "$0")/../../.."
V=investigations/consolidated_validation_2026-10-29; RG=$V/scripts/retry_goal.sh
source /opt/ros/jazzy/setup.bash; source install/setup.bash
bash investigations/navfn_mppi_evaluation_2026-10-26/scripts/launch_profile.sh $V/runs/launch_val planner:=navfn structured_routing:=true mppi_path_interpretation:=2d_prefer_forward || exit 1
ros2 action list | grep -E "^/(navigate_to_pose|compute_route)$"; echo "action servers verified"
$RG S1_setup_Home_to_B           240 "nav@8.716@2.705@0.0@default"      8.716 2.705 0.0
$RG T1_routed_B_to_Astaging      240 "nav@0.099@-0.175@0.0@bt"          0.099 -0.175 0.0 106,107,108
$RG S2_setup_Astaging_to_B       240 "nav@8.716@2.705@0.0@default"      8.716 2.705 0.0
$RG T2_free_B_to_Home            200 "nav@-7.589@1.2@0.0@default"      -7.589 1.2 0.0
$RG S3_setup_nose_in_pillar      200 "nav@-7.38@0.90@-1.5708@default"  -7.38 0.90 -1.5708
# --- Test 3: temporary rear obstacle (runtime-spawned static box, removed afterwards; world file untouched)
gz service -s /world/warehouse_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"test_rear_box\"><static>true</static><pose>-7.42 2.30 0.6 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.6 0.6 1.2</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.6 0.6 1.2</size></box></geometry><material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material></visual></link></model></sdf>"'
sleep 8
REAR_BOX="-7.42 2.30 0.30" $RG T3_rear_obstacle 120 "nav@-7.42@3.4@-1.5708@default" -7.42 3.4 -1.5708
gz service -s /world/warehouse_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "test_rear_box" type: MODEL'
sleep 5
$RG S4_setup_to_Home             240 "nav@-7.589@1.2@0.0@default"      -7.589 1.2 0.0
$RG T4_routed_Home_to_B_yaw90    240 "nav@8.716@2.705@1.5708@bt"        8.716 2.705 1.5708 101,103,105
echo ALL_DONE
