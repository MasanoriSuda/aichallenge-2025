# Requirements: Offline disjunction continuation candidate D

## Objective

Evaluate bounded offline candidate D on frozen interaction fingerprint
`7246006054995400977` after A, B and every rough lattice C member failed to
produce a certified ManeuverBundle.

For each exact behind/side/ahead schedule, compare:

1. one cold seven-state SQP evaluation with the final hard disjunction; and
2. the same final problem after a bounded constraint-continuation sequence.

## Invariant under test

If the final problem is feasible but the direct single-SQP path cannot reach
it, intermediate SQP solutions may warm-start progressively stronger versions
of the same disjunction.  Only the final unrelaxed problem and the existing
exact wall/dynamic certificates may form a ManeuverBundle.

## Constraints

- Production authority, controller code path and runtime configuration remain
  unchanged.
- Final wall/opponent model, clearances, solver policy, horizon, weights and
  terminal semantics are identical to A/B/C.
- Intermediate continuation artifacts are never publishable and never count
  as success.
- Every intermediate and final snapshot has an immutable fingerprint.
- A direct final solve is measured separately; it must not be credited to
  continuation.
- Failure of every D member remains `Unknown` unless an explicit bounded
  physical infeasibility certificate is obtained.

## Definition of Done

- A failure-first test demonstrates that the direct final problem and its
  continuation history are distinguishable.
- The continuation parameter is shadow-only and absent from production.
- Both homotopies and all valid side/ahead schedules are evaluated.
- The final candidate is accepted only after unchanged exact wall, timed
  dynamic-obstacle and terminal-successor proofs.
- Focused tests, full package tests and `make autoware-build` pass.
- The result and next architectural decision are recorded centrally.
