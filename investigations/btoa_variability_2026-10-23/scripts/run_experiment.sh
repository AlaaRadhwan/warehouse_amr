#!/bin/bash
# One instrumented B->A run with the robot SPAWNED at Station B.
# usage: run_experiment.sh RUN_ID MULTIPLIER      e.g. run_experiment.sh p01_m1.0 1.0
# Never overwrites an existing run directory. Always finalizes the bag and shuts the simulation down (trap).
HERE="$(cd "$(dirname "$0")" && pwd)"; INV="$(dirname "$HERE")"; REPO="$(cd "$INV/../.." && pwd)"
RUN_ID=${1:?run id}; MULT=${2:?multiplier}
source "$INV/config/experiment.env"
RUN="$INV/runs/$RUN_ID"
[ -e "$RUN" ] && { echo "refusing to overwrite $RUN"; exit 9; }
mkdir -p "$RUN"
free_gb=$(df --output=avail -BG /home | tail -1 | tr -dc 0-9)
echo "free on /home: ${free_gb} GB" | tee "$RUN/disk.txt"
[ "$free_gb" -lt "$MIN_FREE_GB" ] && { echo "ABORT: /home below ${MIN_FREE_GB} GB"; echo '{"status":"INFRA_ABORT_DISK"}' > "$RUN/run_status.json"; exit 8; }
source /opt/ros/jazzy/setup.bash; source "$REPO/install/setup.bash"
NV="$REPO/calibration/nav_validation"
status() { echo "{\"status\":\"$1\",\"detail\":\"$2\"}" > "$RUN/run_status.json"; }
cleanup() {
  # SIGTERM, not SIGINT: background jobs of a non-interactive shell ignore SIGINT (that is why older bags needed reindexing)
  kill -TERM ${CLIENTPID:-} 2>/dev/null
  { echo "--- top CPU at end"; ps -eo pcpu,pmem,args --sort=-pcpu | head -9 | cut -c1-110; cat /proc/loadavg; } > "$RUN/load_snapshot_end.txt" 2>&1
  kill -TERM ${BAGPID:-} ${GTPID:-} ${MONPID:-} 2>/dev/null
  for i in $(seq 1 40); do kill -0 ${BAGPID:-0} 2>/dev/null || break; sleep 1; done     # rosbag2 must finalize metadata.yaml
  kill -KILL ${BAGPID:-} 2>/dev/null
  [ -d "$RUN/bag" ] && [ ! -f "$RUN/bag/metadata.yaml" ] && { echo "bag metadata missing -> reindex" | tee -a "$RUN/shutdown.log"; ros2 bag reindex -s mcap "$RUN/bag" >> "$RUN/shutdown.log" 2>&1; }
  "$NV/kill_sim.sh" | tee -a "$RUN/shutdown.log"
  date +%s.%N > "$RUN/wall_end.txt"
}
trap cleanup EXIT
date +%s.%N > "$RUN/wall_start.txt"

# ---- provenance snapshot (read-only)
{ echo "run_id=$RUN_ID multiplier=$MULT"; echo "git_head=$(git -C "$REPO" rev-parse --short HEAD) branch=$(git -C "$REPO" branch --show-current)"
  echo "--- git status --short"; git -C "$REPO" status --short
  echo "--- sha256 (first 16)"; sha256sum "$REPO/src/amr_control/config/controllers.yaml" "$REPO/src/amr_navigation/config/nav2_params.yaml" "$REPO/src/amr_missions/config/stations.yaml" | cut -c1-16,65-
  echo "--- uname"; uname -srm; echo "--- nproc"; nproc; echo "--- loadavg at start"; cat /proc/loadavg; echo "--- nav2_mppi"; dpkg -l ros-jazzy-nav2-mppi-controller | tail -1 | awk '{print $2,$3}'; } > "$RUN/provenance.txt" 2>&1
{ echo "--- top CPU at start (desktop/other load is a timing confounder)"; ps -eo pcpu,pmem,args --sort=-pcpu | head -9 | cut -c1-110; cat /proc/loadavg; free -m | head -2; } > "$RUN/load_snapshot_start.txt" 2>&1
cp "$REPO/src/amr_navigation/config/nav2_params.yaml" "$RUN/nav2_params.snapshot.yaml"
python3 "$HERE/make_variants.py" "$RUN/variant" "$MULT" > "$RUN/variant.log" 2>&1 || { status INFRA_VARIANT "make_variants failed"; exit 5; }

# ---- launch
"$NV/kill_sim.sh" > "$RUN/preclean.log"
grep -q "sim processes: clean" "$RUN/preclean.log" || { status INFRA_PRECLEAN "leftover processes"; exit 6; }
setsid nohup ros2 launch "$RUN/variant/bringup.launch.py" > "$RUN/launch.log" 2>&1 &
ready=0; t_ready0=$SECONDS
# every ROS CLI probe has its own timeout (an un-timed `ros2 lifecycle get` once hung the whole pilot for 28 min when Nav2's
# lifecycle bringup failed); readiness has a hard 150 s wall deadline and aborts at once if Nav2 reports a failed bringup
while [ $((SECONDS - t_ready0)) -lt 150 ]; do
  sleep 2
  if grep -q "failed to send response to /.*/change_state" "$RUN/launch.log"; then status INFRA_LIFECYCLE_RESPONSE_LOST "an RMW service response to a lifecycle change_state was dropped at startup (see launch.log); Nav2 would stall"; echo "LIFECYCLE RESPONSE LOST"; exit 4; fi
  if grep -q "Failed to bring up all requested nodes" "$RUN/launch.log"; then status INFRA_NAV2_BRINGUP_FAILED "lifecycle manager aborted bringup (see launch.log)"; echo "NAV2 BRINGUP FAILED"; exit 4; fi
  ok=1
  for nd in bt_navigator controller_server planner_server amcl; do [[ "$(timeout 5 ros2 lifecycle get /$nd 2>/dev/null)" == "active"* ]] || ok=0; done
  timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" || ok=0
  timeout 5 ros2 action list 2>/dev/null | grep -q "^/navigate_to_pose$" || ok=0
  [ $ok = 1 ] && { ready=1; break; }
done
i=$(( (SECONDS - t_ready0) / 2 ))
[ $ready = 1 ] || { status INFRA_NOT_READY "Nav2/controllers not active within 150 s"; echo "NOT READY"; exit 4; }
timeout 15 ros2 topic echo --once /amcl_pose > "$RUN/amcl_pose_at_ready.txt" 2>&1 || { status INFRA_NO_AMCL "no /amcl_pose"; exit 4; }
timeout 15 ros2 topic echo --once /odom > /dev/null 2>&1 || { status INFRA_NO_ODOM "no /odom"; exit 4; }
echo "ready after ~$((i*2)) s wall" | tee "$RUN/ready.log"
sleep "$SETTLE_S"
mkdir -p "$RUN/param_dump"
export RUN
timeout 10 ros2 node list 2>/dev/null | grep -v -E "rviz|transform_listener|_ros2cli|daemon|btoa|sysmon" | \
  xargs -P 8 -n 1 bash -c 'nd=$1; timeout -k 2 15 ros2 param dump "$nd" > "$RUN/param_dump/$(echo $nd | tr / _).yaml" 2>/dev/null' _
echo "driver_version=3 (v2 + early abort on a lost lifecycle response; v2 = parallel parameter dump; v1 = sequential dump, runs p01-p04)" > "$RUN/driver_version.txt"
python3 -c "import re,sys;d=open('$RUN/param_dump/_diff_drive_controller.yaml').read();print(re.findall(r'wheel_separation_multiplier: .*',d))" | tee "$RUN/loaded_multiplier.txt"

# ---- recorders
python3 "$HERE/sysmon.py" "$RUN/sysmon.csv" > "$RUN/sysmon.log" 2>&1 & MONPID=$!
python3 "$HERE/gt_recorder.py" "$RUN/gt.csv" > "$RUN/gt_recorder.log" 2>&1 & GTPID=$!
ros2 bag record --include-hidden-topics --qos-profile-overrides-path "$NV/qos_overrides.yaml" -o "$RUN/bag" \
    --topics $(grep -v '^#' "$INV/config/topics.txt" | tr '\n' ' ') > "$RUN/recorder.log" 2>&1 & BAGPID=$!
sleep 8
[ "$(wc -l < "$RUN/gt.csv")" -gt 20 ] || { status INFRA_NO_GT "ground-truth recorder produced no data"; exit 7; }

# ---- goal
python3 "$HERE/btoa_client.py" "$RUN" --sim-limit "$SIM_LIMIT" --wall-limit "$WALL_LIMIT" --post-trigger "$POST_TRIGGER" > "$RUN/client.log" 2>&1 &
CLIENTPID=$!
wait $CLIENTPID; echo "client exit: $?" | tee "$RUN/client_exit.txt"; CLIENTPID=
sleep 6          # keep recording the settle / post-cancel behaviour
status COMPLETED "client finished; see client_result.json"
