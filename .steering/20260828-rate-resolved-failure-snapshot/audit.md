# Audit: Rate-resolved failure snapshot

## Observed phenomenon

In `output/20260828-001351/d1/autoware.log`, ShiftOut was admitted at decision
4246 with a certified seven-state artifact.  Before Pass, later problems were
rejected by exact lateral-bound proof and dynamic-obstacle QP solves.  At
decision 4455 ShiftOut moved to FollowPrepare because of SafetyBrake.  A new
opposite-side generation then resumed ShiftOut, but no Pass/Return transition
was observed before the run was stopped.

## Current classification

Unknown, but the first numerical branch is now bounded more tightly.

Run `output/20260828-003105` captured decision 1152 as an immutable exact QP:

- intent: `ShiftOut`;
- source: right-side `receding-prefix` pre-entry candidate;
- pipeline stage: initial seven-state QP;
- production result: OSQP maximum iterations at 4,000 iterations;
- worst physical row: stage-zero steering-rate input box;
- production total solve time: about 31 ms including the live context;
- offline warm replay: the same maximum-iteration rejection, same row and
  normalized violation, about 11.7 ms;
- offline cold replay: the same maximum-iteration rejection, same row and
  normalized violation, about 11.6 ms.

This refutes stale warm-start state as the cause of this particular failure
and shows that removing live scheduling alone cannot make the frozen convex
problem solve.  It does **not** distinguish bad candidate geometry from a
single-SQP limitation.  It is also not the required Pass/Return architecture
snapshot: the failed initial pre-entry problem did not request physical wall
refinement and consequently carried no wall-grid payload or terminal physical
certificate.

## Production impact

None.  The recorder runs only after production has already rejected an
Overtake solve, deduplicates by `(intent, pipeline stage, failure outcome)`,
and has no ROS publisher or authority API.

## Root-cause boundary established

The observed failure is not currently evidence for a Mission resume rule,
clearance change, solver tolerance change or parameter tuning.  A rough
alternate candidate (C) and bounded multi-SQP feasibility solve (D) must be
run against a complete current-world snapshot before production changes.

## Verification

- `make autoware-build`: 25 packages passed.
- package CTest: 49/49 targets passed.
- snapshot unit tests: write/load, deduplication, warm replay, cold replay and
  non-Overtake rejection passed.
- `make dev2`: snapshot path appeared in the production decision log.
- exact warm/cold replay: both reproduced the frozen failure.
