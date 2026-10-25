#!/bin/bash
# Controlled B -> A navigation run. Nav2 / robot configuration is used exactly as installed.
#   1. launch ONE full simulation + Nav2 (amr_bringup simulation_navigation.launch.py)
#   2. Home -> Station B (not recorded)
#   3. record + send NavigateToPose to the Station A staging pose, with a pathological-rotation / no-progress watchdog (nav_watchdog.py)
#   4. ALWAYS shut the simulation down (trap)
# usage: run_b_to_a.sh LABEL

HERE="$(cd "$(dirname "$0")" && pwd)"; REPO="$(cd "$HERE/../.." && pwd)"
LABEL=${1:?label}; RUN="$HERE/runs/$(date +%Y%m%d_%H%M%S)_$LABEL"; mkdir -p "$RUN"
source /opt/ros/jazzy/setup.bash; source "$REPO/install/setup.bash"
cleanup() {
  kill -INT ${BAGPID:-} ${GTPID:-} 2>/dev/null
  for i in $(seq 1 30); do kill -0 ${BAGPID:-0} 2>/dev/null || break; sleep 1; done   # let rosbag2 finalize metadata.yaml
  "$HERE/kill_sim.sh" | tee -a "$RUN/shutdown.log"
}
trap cleanup EXIT
"$HERE/kill_sim.sh" | tee "$RUN/preclean.log"          # no orphans may share the gz transport
ps -eo args | grep -c '^gz sim server' | sed 's/^/gz servers before launch: /' | tee -a "$RUN/preclean.log"

setsid nohup ros2 launch ${LAUNCH_OVERRIDE:-amr_bringup simulation_navigation.launch.py} > "$RUN/launch.log" 2>&1 &
for i in $(seq 1 90); do
  sleep 2
  s1=$(ros2 lifecycle get /bt_navigator 2>/dev/null); s2=$(ros2 lifecycle get /controller_server 2>/dev/null)
  [[ "$s1" == "active"* && "$s2" == "active"* ]] && ros2 action list 2>/dev/null | grep -q "^/navigate_to_pose$" && break
done
echo "nav ready after ~$((i*2)) s: bt_navigator=$s1 controller_server=$s2" | tee "$RUN/ready.log"
echo "gz servers: $(ps -eo args | grep -c '^gz sim server')" | tee -a "$RUN/ready.log"
for p in wheel_separation wheel_separation_multiplier wheel_radius; do echo "$p: $(ros2 param get /diff_drive_controller $p)"; done | tee "$RUN/controller_params.txt"
if [ -n "${MULT_OVERRIDE:-}" ]; then      # CONTROL RUNS ONLY: runtime (non-persistent) override, no file is edited
  ros2 param set /diff_drive_controller wheel_separation_multiplier "$MULT_OVERRIDE" | tee -a "$RUN/controller_params.txt"
  echo "RUNTIME OVERRIDE wheel_separation_multiplier=$MULT_OVERRIDE -> $(ros2 param get /diff_drive_controller wheel_separation_multiplier)" | tee -a "$RUN/controller_params.txt"
fi
sleep 8

python3 "$HERE/nav_goal.py" station_b --out "$RUN" --label home_to_b --no-watchdog --timeout 300 || { echo "HOME->B FAILED; stopping"; exit 3; }
sleep 6

TOPICS="/clock /tf /tf_static /plan /plan_smoothed /unsmoothed_plan /transformed_global_plan /optimal_trajectory ${WITH_TRAJECTORIES:+/trajectories} \
 /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /diff_drive_controller/cmd_vel_out /diff_drive_controller/odom /joint_states \
 /imu/data_raw /odom /amcl_pose /collision_monitor_state /speed_limit /behavior_tree_log \
 /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status"
ros2 bag record --use-sim-time --include-hidden-topics --qos-profile-overrides-path "$HERE/qos_overrides.yaml" \
   -o "$RUN/bag" --topics $TOPICS > "$RUN/recorder.log" 2>&1 &
BAGPID=$!
python3 "$HERE/record_gt.py" "$RUN/gt.csv" > "$RUN/gt_recorder.log" 2>&1 &
GTPID=$!
sleep 6
python3 "$HERE/nav_goal.py" staging_a --out "$RUN" --label b_to_a --timeout 240
echo "nav_goal exit: $?" | tee "$RUN/client_exit.txt"
sleep 6          # keep recording the settle / post-cancel behaviour
echo DONE > "$RUN/DONE"
