#!/bin/bash
# usage: launch_profile.sh LOGDIR [ros2 launch args...]
# Starts the sim and waits until the NAVIGATION lifecycle manager reports active. Nav2 startup occasionally hangs
# (a lifecycle change_state response is lost: "failed to send response ... (timeout)"), so a hung attempt is
# torn down and retried (max 3); each failed attempt's log is kept as launch.attemptN_FAILED.log.
cd "$(dirname "$0")/../../.."
L=$1; shift; mkdir -p "$L"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
for attempt in 1 2 3; do
  bash ../../calibration/nav_validation/kill_sim.sh >/dev/null
  setsid nohup ros2 launch amr_bringup simulation_navigation.launch.py "$@" > "$L/launch.log" 2>&1 &
  for i in $(seq 1 25); do grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$L/launch.log" && break; sleep 3; done
  if grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$L/launch.log"; then
    sleep 15; ok=1
    for n in controller_server bt_navigator route_server; do s=$(ros2 lifecycle get /$n | head -1); echo "$n: $s"; [[ $s == active* ]] || ok=0; done
    [ $ok = 1 ] && { echo "launch ok (attempt $attempt)"; exit 0; }
  fi
  echo "attempt $attempt: navigation did not come up (lost lifecycle response?)"; cp "$L/launch.log" "$L/launch.attempt${attempt}_FAILED.log"
done
echo "NAVIGATION NOT ACTIVE after 3 attempts"; exit 1
