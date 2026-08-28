# Design

## Hypotheses

1. Live V2X/tactical candidate construction is synchronously recomputing work
   that belongs to the asynchronous topology worker.
2. A causal pre-entry snapshot is deep-copied or rebuilt on every callback.
3. Retained proof itself is too expensive.
4. CPU contention exists, but no individual production region exceeds the
   budget.

Each hypothesis predicts a different dominant timing region.  The first change
therefore adds observation only.

## Timing contract

`init_problem()` records monotonic checkpoints around:

- base geometry/setup;
- live `evaluate_v2x_behavior()`;
- wall prewarm and pre-entry-result consumption;
- dynamic/gap planning up to the overtake-line boundary;
- `update_overtake_line()`;
- final QP/problem assembly.

`rate_resolved_normal_production_control()` separately records:

- pre-entry execution draft construction;
- normal successor draft construction;
- current-world retained proof and atomic intent join;
- publication-successor preparation.

Only over-budget production callbacks emit the structured diagnostic.  The
measurement cannot affect path, command, certification or authority selection.

## Decision after measurement

- dominant live behavior/gap planning: move topology/candidate ownership to
  the existing latest-only worker and keep only immutable-result consumption
  on the callback;
- dominant pre-entry snapshot: construct it only at the existing async
  submission boundary, not every command cycle;
- dominant retained proof: split proof preparation from the bounded current
  state join without weakening the exact physical certificate;
- no dominant region: inspect executor/CPU scheduling before code changes.

## Measured decision

Two bounded `make dev2` runs reached the start-grid transition.  The second
run (`output/20260828-205642`) reproduced the frozen overrun with direct
attribution:

- decision 1945: total problem assembly 66.060 ms;
- live behavior: 65.931 ms;
- all other measured problem regions combined: 0.129 ms;
- following window: 17 callbacks/s, 15 overruns/s, MPC region average
  55.330 ms;
- decision 1961: live behavior 68.479 ms.

The expensive path was not an unowned computation.  The latest-only worker
already owns ordinary side/corridor/Mission construction, but
`defer_live_tactical_generation()` explicitly exempted start-grid breakout.
Consequently the 40 Hz callback synchronously generated both sides and the
complete Mission while the worker architecture existed alongside it.

The correction removes that start-grid ownership exception.  Live target,
risk and state observation remain synchronous; side/corridor/Mission
construction is deferred exactly as it is for every other async tactical
scene.  No timing, clearance, solver or authority threshold changes.
