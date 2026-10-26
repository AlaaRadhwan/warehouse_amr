# NavFn + MPPI evaluation (EXPERIMENTAL) — 2026-10-26
Report: `docs/NAVFN_MPPI_EVALUATION.md`. `scripts/`: launch helper with startup-hang retry, per-goal runner, metrics, recompute. `runs/<profile>_<goal>/`: `result.json`, `metrics.json`
(raw `gt.csv`, `plog.jsonl` and launch logs are local only). `runs/baseline_recomputed/`: the State Lattice baseline runs from `investigations/structured_routing_2026-10-26` re-analysed with the same metrics.

Second report: `docs/LATTICE_2D_FORWARD_EVALUATION.md` (State Lattice + 2D MPPI + forward-only; runs `lat2dfwd_*`, `latRev2dfwd_*`, `latfwd_*`, `latfwdBaseMPPI_*`; `latfwd_*_INVALID_*` are excluded runs).
