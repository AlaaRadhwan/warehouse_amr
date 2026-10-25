#!/bin/bash
# Pilot: 3 runs per arm, arms interleaved (ABABAB) to spread slow drift in machine load across both arms.
# An infrastructure failure (run_status INFRA_*) is retried ONCE under a new id; the failed attempt is kept.
# usage: pilot.sh [START_INDEX]   (1-based; 1 = fresh pilot, N = resume at run N, appending to runs/pilot_order.txt)
HERE="$(cd "$(dirname "$0")" && pwd)"; INV="$(dirname "$HERE")"
ORDER=(1.0 0.92754 1.0 0.92754 1.0 0.92754); START=${1:-1}
[ "$START" = 1 ] && echo "# pilot order (start $(date -Is))" > "$INV/runs/pilot_order.txt" || echo "# resumed at run $START $(date -Is)" >> "$INV/runs/pilot_order.txt"
for i in "${!ORDER[@]}"; do
  [ $((i+1)) -lt "$START" ] && continue
  m=${ORDER[$i]}; id=$(printf "p%02d_m%s" $((i+1)) $m)
  [ -e "$INV/runs/$id" ] && id="${id}_retry1"            # a failed attempt already exists under the plain id: keep it, use the retry id
  echo "$id $m $(date -Is)" >> "$INV/runs/pilot_order.txt"
  "$HERE/run_experiment.sh" "$id" "$m" > "$INV/runs/$id.out" 2>&1
  if grep -q '"status":"INFRA' "$INV/runs/$id/run_status.json" 2>/dev/null && [[ "$id" != *_retry1 ]]; then
    id2="${id}_retry1"; echo "$id2 $m $(date -Is) (retry of $id)" >> "$INV/runs/pilot_order.txt"
    "$HERE/run_experiment.sh" "$id2" "$m" > "$INV/runs/$id2.out" 2>&1
  fi
done
echo "# pilot done $(date -Is)" >> "$INV/runs/pilot_order.txt"
