#!/bin/bash
# usage: demo_run_b.sh problem|fixed   -- robot SPAWNS at station B; only the B->A goal is sent. No bag, no watchdog; light gt.csv to scratchpad.
MODE=${1:?problem|fixed}
REPO=/home/alaa-radhwan/Projects/warehouse_amr; NV=$REPO/calibration/nav_validation
S=/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/5b1b5d02-6193-49b3-a41c-6e24ff040f58/scratchpad
OUT=$S/demoB_${MODE}_$(date +%H%M%S); mkdir -p $OUT; echo "out: $OUT"
source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash
cd $NV
trap 'kill -INT ${GTPID:-} 2>/dev/null; $NV/kill_sim.sh | tee -a $OUT/shutdown.log' EXIT
$NV/kill_sim.sh > $OUT/preclean.log
L="$S/uncal/bringup_fixed_atB.launch.py"; [ "$MODE" = problem ] && L="$S/uncal/bringup_uncal_atB.launch.py"
setsid nohup ros2 launch $L > $OUT/launch.log 2>&1 &
for i in $(seq 1 90); do sleep 2
  s1=$(ros2 lifecycle get /bt_navigator 2>/dev/null); s2=$(ros2 lifecycle get /controller_server 2>/dev/null)
  [[ "$s1" == "active"* && "$s2" == "active"* ]] && ros2 action list 2>/dev/null | grep -q "^/navigate_to_pose$" && break; done
[[ "$s1" == "active"* && "$s2" == "active"* ]] || { echo "NAV2 NOT READY: bt=$s1 ctrl=$s2"; exit 4; }
echo "ready ~$((i*2))s"; echo "multiplier: $(ros2 param get /diff_drive_controller wheel_separation_multiplier)" | tee $OUT/params.txt
sleep 10
timeout 8 ros2 topic echo --once --field pose.pose /amcl_pose 2>&1 | tr '\n' ' ' | tee $OUT/start_pose.txt; echo
python3 $NV/record_gt.py $OUT/gt.csv > $OUT/gt_recorder.log 2>&1 &
GTPID=$!; sleep 3
echo "B->A goal sent $(date +%T)"
timeout ${CAP:-150} python3 $NV/nav_goal.py staging_a --out $OUT --label b_to_a --no-watchdog --timeout 240
echo "nav_goal exit: $? at $(date +%T)"; sleep 5
