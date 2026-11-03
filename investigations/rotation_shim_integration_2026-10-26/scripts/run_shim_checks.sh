#!/bin/bash
# Rotation Shim integration checks: ONE simulation, robot spawned at Station B, goals run in sequence by shim_client.py.
# Lightweight recording only (command chain, odom, plans, tf; no scan / costmaps / candidate trajectories).
# usage: [OVERRIDES="ANCHOR|KEY|VALUE;..."] run_shim_checks.sh RUN_ID [GOALSET]      (GOALSET: full | p2)
# OVERRIDES makes a TEMPORARY params file for this run only (see make_params.py); src/ is never modified.
HERE="$(cd "$(dirname "$0")" && pwd)"; INV="$(dirname "$HERE")"; REPO="$(cd "$INV/../.." && pwd)"
BTOA="$REPO/investigations/btoa_variability_2026-10-23/scripts"; NV="$REPO/calibration/nav_validation"
RUN_ID=${1:?run id}; GOALSET=${2:-full}; RUN="$INV/runs/$RUN_ID"
[ -e "$RUN" ] && { echo "refusing to overwrite $RUN"; exit 9; }
mkdir -p "$RUN"
free_gb=$(df --output=avail -BG /home | tail -1 | tr -dc 0-9); echo "free on /home: ${free_gb} GB" | tee "$RUN/disk.txt"
[ "$free_gb" -lt 2 ] && { echo "ABORT: low disk"; exit 8; }
source /opt/ros/jazzy/setup.bash; source "$REPO/install/setup.bash"
status() { echo "{\"status\":\"$1\",\"detail\":\"$2\"}" > "$RUN/run_status.json"; }
cleanup() {
  kill -TERM ${CLIENTPID:-} 2>/dev/null
  kill -TERM ${BAGPID:-} ${GTPID:-} 2>/dev/null
  for i in $(seq 1 40); do kill -0 ${BAGPID:-0} 2>/dev/null || break; sleep 1; done
  kill -KILL ${BAGPID:-} 2>/dev/null
  [ -d "$RUN/bag" ] && [ ! -f "$RUN/bag/metadata.yaml" ] && ros2 bag reindex -s mcap "$RUN/bag" >> "$RUN/shutdown.log" 2>&1
  "$NV/kill_sim.sh" | tee -a "$RUN/shutdown.log"
}
trap cleanup EXIT
{ echo "run_id=$RUN_ID goalset=$GOALSET"; echo "git_head=$(git -C "$REPO" rev-parse --short HEAD) branch=$(git -C "$REPO" branch --show-current)"
  git -C "$REPO" status --short | head -20; sha256sum "$REPO/src/amr_navigation/config/nav2_params.yaml" "$REPO/src/amr_control/config/controllers.yaml" | cut -c1-16,65-
  dpkg -l ros-jazzy-nav2-rotation-shim-controller ros-jazzy-nav2-mppi-controller | tail -2 | awk '{print $2,$3}'; cat /proc/loadavg; } > "$RUN/provenance.txt" 2>&1
cp "$REPO/src/amr_navigation/config/nav2_params.yaml" "$RUN/nav2_params.snapshot.yaml"
echo "OVERRIDES=${OVERRIDES:-<none>}" >> "$RUN/provenance.txt"
# start at Station B with the repo's calibrated multiplier (generated variant differs from the repo file by no line)
python3 "$BTOA/make_variants.py" "$RUN/variant" 0.92754 > "$RUN/variant.log" 2>&1 || { status INFRA_VARIANT "make_variants failed"; exit 5; }
# temporary params file for this run (identical to the repo file when OVERRIDES is empty), wired into the generated bringup launch
IFS=';' read -ra OVR <<< "${OVERRIDES:-}"
python3 "$HERE/make_params.py" "$RUN/nav2_params.run.yaml" "${OVR[@]}" > "$RUN/params_override.log" 2>&1 || { status INFRA_PARAMS "make_params failed"; exit 5; }
python3 - "$RUN" <<'PY' || { status INFRA_VARIANT "bringup patch failed"; exit 5; }
import sys; from pathlib import Path
run = Path(sys.argv[1]); b = (run / "variant/bringup.launch.py").read_text()
old = """    nav2_params_file = os.path.join(
        navigation_share,
        "config",
        "nav2_params.yaml",
    )"""
assert old in b; (run / "variant/bringup.launch.py").write_text(b.replace(old, f'    nav2_params_file = "{run / "nav2_params.run.yaml"}"'))
PY
"$NV/kill_sim.sh" > "$RUN/preclean.log"; grep -q "sim processes: clean" "$RUN/preclean.log" || { status INFRA_PRECLEAN "leftover processes"; exit 6; }
setsid nohup ros2 launch "$RUN/variant/bringup.launch.py" > "$RUN/launch.log" 2>&1 &
ready=0; t0=$SECONDS
while [ $((SECONDS - t0)) -lt 150 ]; do
  sleep 2
  grep -q "failed to send response to /.*/change_state" "$RUN/launch.log" && { status INFRA_LIFECYCLE_RESPONSE_LOST "see launch.log"; echo "LIFECYCLE RESPONSE LOST"; exit 4; }
  grep -q "Failed to bring up all requested nodes" "$RUN/launch.log" && { status INFRA_NAV2_BRINGUP_FAILED "see launch.log"; echo "NAV2 BRINGUP FAILED"; exit 4; }
  ok=1
  for nd in bt_navigator controller_server planner_server amcl; do [[ "$(timeout 5 ros2 lifecycle get /$nd 2>/dev/null)" == "active"* ]] || ok=0; done
  timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" || ok=0
  timeout 5 ros2 action list 2>/dev/null | grep -q "^/navigate_to_pose$" || ok=0
  [ $ok = 1 ] && { ready=1; break; }
done
[ $ready = 1 ] || { status INFRA_NOT_READY "not active within 150 s"; echo "NOT READY"; exit 4; }
echo "ready after ~$((SECONDS - t0)) s wall" | tee "$RUN/ready.log"
timeout 15 ros2 topic echo --once /amcl_pose > "$RUN/amcl_pose_at_ready.txt" 2>&1 || { status INFRA_NO_AMCL "no /amcl_pose"; exit 4; }
sleep 10
mkdir -p "$RUN/param_dump"; export RUN
DUMPPIDS=""
for nd in /controller_server /velocity_smoother /planner_server /diff_drive_controller /bt_navigator; do
  timeout -k 2 20 ros2 param dump $nd > "$RUN/param_dump/$(echo $nd | tr / _).yaml" 2>/dev/null & DUMPPIDS="$DUMPPIDS $!"
done; wait $DUMPPIDS      # NOT a bare `wait`: that would also wait for the background ros2 launch forever
python3 "$BTOA/gt_recorder.py" "$RUN/gt.csv" > "$RUN/gt_recorder.log" 2>&1 & GTPID=$!
ros2 bag record --include-hidden-topics --qos-profile-overrides-path "$NV/qos_overrides.yaml" -o "$RUN/bag" \
  --topics $(grep -v '^#' "$INV/config/topics.txt" | tr '\n' ' ') > "$RUN/recorder.log" 2>&1 & BAGPID=$!
sleep 8
[ "$(wc -l < "$RUN/gt.csv")" -gt 20 ] || { status INFRA_NO_GT "no ground truth"; exit 7; }
python3 "$HERE/shim_client.py" "$RUN" --goalset "$GOALSET" > "$RUN/client.log" 2>&1 &
CLIENTPID=$!; wait $CLIENTPID; echo "client exit: $?" | tee "$RUN/client_exit.txt"; CLIENTPID=
sleep 5; status COMPLETED "see client_result.json"
