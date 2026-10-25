#!/bin/bash
# Extension of the pilot: N_PAIRS pairs of runs, arms interleaved (1.0, 0.92754, ...), ids e01_m1.0, e02_m0.92754, ...
# One retry on infrastructure failure, same rules as pilot.sh. usage: extend.sh N_PAIRS
HERE="$(cd "$(dirname "$0")" && pwd)"; INV="$(dirname "$HERE")"; N=${1:-4}
echo "# extension: $N pairs (start $(date -Is))" >> "$INV/runs/pilot_order.txt"
for i in $(seq 1 $((2*N))); do
  if [ $((i % 2)) = 1 ]; then m=1.0; else m=0.92754; fi
  id=$(printf "e%02d_m%s" $i $m); [ -e "$INV/runs/$id" ] && id="${id}_retry1"
  echo "$id $m $(date -Is)" >> "$INV/runs/pilot_order.txt"
  "$HERE/run_experiment.sh" "$id" "$m" > "$INV/runs/$id.out" 2>&1
  if grep -q '"status":"INFRA' "$INV/runs/$id/run_status.json" 2>/dev/null && [[ "$id" != *_retry1 ]]; then
    id2="${id}_retry1"; echo "$id2 $m $(date -Is) (retry of $id)" >> "$INV/runs/pilot_order.txt"
    "$HERE/run_experiment.sh" "$id2" "$m" > "$INV/runs/$id2.out" 2>&1
  fi
done
echo "# extension done $(date -Is)" >> "$INV/runs/pilot_order.txt"
