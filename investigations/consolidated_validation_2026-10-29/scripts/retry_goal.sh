#!/bin/bash
# Run one goal via the existing run_goal.sh; if the goal was NOT acknowledged by the action server (client-side, no motion),
# label that attempt INVALID and retry (max 2 retries). usage: retry_goal.sh NAME TIMEOUT GOAL GX GY GYAW [EDGES]
RG="$(dirname "$0")/run_goal.sh"; NAME=$1; shift
RUNS="$(dirname "$0")/../runs"
for k in 0 1 2; do
  $RG $NAME "$@" | tail -2
  if grep -q '"accepted": true' $RUNS/$NAME/result.json 2>/dev/null; then exit 0; fi
  mv $RUNS/$NAME $RUNS/${NAME}_INVALID_goal_not_acknowledged_try$k; echo "goal not acknowledged -> $NAME try $k labelled INVALID"
  sleep 5
done
