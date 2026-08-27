# Requirements

## Objective

Run the first evidence-driven Slice 7 experiment against structural baseline
`b273d56d`.  Reduce the canonical MPCC horizon family only, then compare
real-time tail behavior and complete Overtake phase execution.

## Baseline evidence

Run `output/20260828-044759` reported 141 one-second callback windows, 102
individual 25 ms overruns, a maximum callback time of 56.647 ms, and a maximum
MPCC time of 56.310 ms.  The overrun clusters coincide with full 20-stage
ShiftOut/Pass solves.  Conditional post-refinement was not requested in the
sampled clusters, so changing its deadline would not address the measured
work.

## Experiment

- A: `mpc.N = 20` (frozen structural baseline).
- B: `mpc.N = 16`.
- Keep control rate, SQP formulation, weights, clearances, solver tolerances,
  candidate generation, authority, and fallback behavior unchanged.
- Apply the accepted value consistently to local and cloud configurations.

## Acceptance

- No architecture/source-contract regression.
- Canonical normal authority remains the seven-state MPCC.
- Callback overrun count and maximum MPCC cycle time improve materially.
- At least one `ShiftOut -> Pass` occurs, or all failed entries have a typed
  physical/candidate cause rather than stale/late authority.
- No new ShiftOut/Pass wall Recovery or solver-collapse loop.

If runtime tails do not improve or dynamic execution regresses, restore
`N=20`; do not compensate with another parameter family in this experiment.
