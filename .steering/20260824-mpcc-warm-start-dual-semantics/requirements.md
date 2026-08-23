# Requirements: MPCC warm-start dual semantics

## Objective

Remove the semantic mismatch in shifted MPC/MPCC dual warm starts. A new
horizon's measured-state equality and first absolute curvature constraint must
not inherit multipliers from prior inter-stage constraints.

## Failure evidence

- Strict Track/Cruise replay: `output/20260824-011002`.
- Domain 1: all 32 failures used a warm start; the following cold cycle was
  certified.
- Domain 2: all 9 failures used a warm start; the following cold cycle was
  certified.
- Rejected rows 210/212/270 decode to first-input acceleration,
  virtual-progress speed and first absolute curvature-rate constraints for
  `N=20`.

## In scope

- Correct the pure `shift_mpc_warm_start` dual mapping.
- Preserve primal stage shifting and every same-semantic dual stage block.
- Zero only new-horizon dual rows that have no same-semantic predecessor.
- Add exact unit tests for the layout.
- Run full package/build and a bounded closed-loop gate before any strict
  Track/Cruise row-policy promotion.

## Out of scope

- Solver tolerance/settings, weights, horizon or clearance tuning.
- New fallback, retry, timeout, lease or authority source.
- Retained-plan eligibility broadening.
- Overtake promotion.

## Acceptance

- Initial-state equality duals are zero after shift.
- First absolute curvature constraint dual is zero after shift.
- Remaining state/input/rate/trailing duals retain their correct stage map.
- Full package tests and build pass.
- Closed-loop evidence does not regress canonical availability.

