# Requirements: proof-authoritative wall pose

## Objective

Replace the hard post-hoc lag/heading wall-pose veto only if the frozen
evidence proves how the racing QP obtains a feasible iterate.  Preserve one
canonical seven-state authority and every exact physical certificate.

## Constraints

- No clearance, tolerance, iteration, timeout, lease, fallback or parameter
  change.
- Direct racing solve and feasibility-first solve must be separated before
  production code changes.
- Exact nonlinear trajectory, swept wall, timed obstacle and terminal
  successor proofs remain mandatory.
- The obsolete hard pose-box authority must be deleted in the same Slice as
  any production promotion.

## Definition of done

1. Same-snapshot direct/feasibility-first evidence identifies the required
   globalization entry.
2. Production has one formulation, not a failure-triggered alternate normal
   controller.
3. Unsafe exact trajectories remain rejected.
4. Frozen corpus, tests, build and dynamic Gate determine promotion.
