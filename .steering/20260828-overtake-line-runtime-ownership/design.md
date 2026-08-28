# Design

## Frozen evidence

Post-fix run `output/20260828-210555` retained approximately 40--41 control
cycles/s, but isolated `update_overtake_line()` calls took 22--28 ms during
`ShiftOut`.

The same windows showed increased physical wall-envelope work.  One window
contained 127 cache misses and 16,877 scanned poses.  This is strong
correlation, but the one-second aggregate cannot distinguish among:

1. baseline/return horizon evaluation;
2. live receding-horizon optimization;
3. solved-execution source revalidation and stitching;
4. other phase-transition or corridor work.

## Observation change

During only the synchronous `update_overtake_line()` call, accumulate:

- total helper calls and time for `evaluate_overtake_line_horizon()`;
- total helper calls and time for `optimize_live_overtake_line_horizon()`;
- solved-execution validation time;
- exact solved-path wall-certificate call count and time;
- physical wall-envelope cache requests, misses and scanned poses.

Emit the combined record only for a call exceeding 20 ms.  Worker snapshots
have independent controller copies and are excluded from the live observation
scope.

## Decision rule

- horizon/receding time plus cache misses dominates: inspect duplicated
  current-world physical envelope construction;
- solved-source/wall-certificate time dominates: inspect redundant proof
  between the canonical certified store and execution bridge;
- neither dominates: split the remaining phase-transition sections before any
  behavior change;
- no reproducible spikes: retain observation and do not optimize speculatively.
