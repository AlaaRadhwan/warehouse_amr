#!/bin/bash
# usage: demo_run.sh problem|fixed   (no bag, no watchdog, no gt recorder)
MODE=${1:?problem|fixed}
REPO=/home/alaa-radhwan/Projects/warehouse_amr; NV=$REPO/calibration/nav_validation
S=/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/5b1b5d02-6193-49b3-a41c-6e24ff040f58/scratchpad
OUT=$S/demo_$MODE; mkdir -p $OUT
source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash
cd $NV
trap '$NV/kill_sim.sh | tee -a $OUT/shutdown.log' EXIT
$NV/kill_sim.sh > $OUT/preclean.log
L="amr_bringup simulation_navigation.launch.py"; [ "$MODE" = problem ] && L="$S/uncal/bringup_uncal.launch.py"
setsid nohup ros2 launch $L > $OUT/launch.log 2>&1 &
for i in $(seq 1 90); do sleep 2
  s1=$(ros2 lifecycle get /bt_navigator 2>/dev/null); s2=$(ros2 lifecycle get /controller_server 2>/dev/null)
  [[ "$s1" == "active"* && "$s2" == "active"* ]] && ros2 action list 2>/dev/null | grep -q "^/navigate_to_pose$" && break; done
[[ "$s1" == "active"* && "$s2" == "active"* ]] || { echo "NAV2 NOT READY: bt=$s1 ctrl=$s2"; exit 4; }
echo "ready ~$((i*2))s"; echo "multiplier: $(ros2 param get /diff_drive_controller wheel_separation_multiplier)" | tee $OUT/params.txt
sleep 8
python3 $NV/nav_goal.py station_b --out $OUT --label home_to_b --no-watchdog --timeout 300 || { echo "HOME->B FAILED"; exit 3; }
sleep 6
echo "B->A goal sent $(date +%T)"
timeout ${CAP:-150} python3 $NV/nav_goal.py staging_a --out $OUT --label b_to_a --no-watchdog --timeout 240
echo "nav_goal exit: $? at $(date +%T)"; sleep 5
