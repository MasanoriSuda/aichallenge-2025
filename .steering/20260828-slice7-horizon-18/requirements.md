# Requirements

## Objective

Determine whether a conservative canonical horizon reduction from 20 to 18
stages improves real-time tails without the terminal-successor regression seen
at 16 stages.

## Constraints

- Change only the `mpc.N` horizon family in local and cloud configurations.
- Preserve model, SQP stages, weights, constraints, clearances, solver
  tolerances, authority, and fallback behavior.
- Compare against structural baseline `b273d56d` and the rejected 16-stage
  evidence in `.steering/20260828-slice7-horizon-ab`.

## Acceptance

- At least one complete `ShiftOut -> Pass -> Return -> Idle` episode.
- No `DynamicMissionWait -> Recovery` caused by missing wall-feasible lateral
  authority.
- No ShiftOut/Pass wall Recovery or repeated maximum-iteration collapse.
- Runtime tail is no worse than the 20-stage baseline and preferably improves
  maximum MPCC time or overrun rate.

Failure of a dynamic criterion restores `N=20` without a compensating tune.
