#!/bin/bash
# Run ONE goal with GT + plan/cmd logging and metrics.  usage: run_goal.sh NAME TIMEOUT GOAL GX GY GYAW [EDGES]
# GOAL = station name or nav@X@Y@YAW@bt|default (see structured_routing_2026-10-26/scripts/mission_run.py)
cd "$(dirname "$0")/../../.."          # worktree root
source /opt/ros/jazzy/setup.bash; source install/setup.bash
NAME=$1; TO=$2; GOAL=$3; GX=$4; GY=$5; GYAW=$6; EDGES=${7:-}
S=investigations/structured_routing_2026-10-26/scripts; D=investigations/navfn_mppi_evaluation_2026-10-26/runs/$NAME; mkdir -p $D
python3 investigations/btoa_variability_2026-10-23/scripts/gt_recorder.py $D/gt.csv >/dev/null 2>&1 & GT=$!
python3 $S/plan_logger.py $D/plog.jsonl >/dev/null 2>&1 & PL=$!
sleep 2
timeout $((TO+60)) python3 $S/mission_run.py $D/result.json "$GOAL:$TO" 2>&1 | tail -1 | cut -c1-400
sleep 3; kill -INT $GT $PL; sleep 1
if [ -n "$EDGES" ]; then
  python3 investigations/navfn_mppi_evaluation_2026-10-26/scripts/analyze_goal.py $D/gt.csv $D/plog.jsonl $GX $GY $GYAW $D/metrics.json $EDGES src/amr_navigation/route_graph/warehouse_route_graph.geojson
else
  python3 investigations/navfn_mppi_evaluation_2026-10-26/scripts/analyze_goal.py $D/gt.csv $D/plog.jsonl $GX $GY $GYAW $D/metrics.json
fi
