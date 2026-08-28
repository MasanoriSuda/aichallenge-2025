# Requirements: diagonal obstacle-guidance comparison

## Objective

Using the frozen Domain 1 failure snapshot from `output/20260828-094214`,
determine whether the remaining Pass failure is caused by the obstacle
candidate representation.  In particular, test whether a certified diagonal
transition exists between the current complete stay-behind and complete
pass-side disjuncts.

## Frozen evidence

- Production baseline: `ae474be3`
- Decision: `1566`, Pass wall-refinement solve rejected
- Interaction fingerprint: `7246006054995400977`
- Snapshot: `000000001566-pass-wall-refinement-solve-rejected/snapshot.yaml`

## Invariants

- Production authority, runtime configuration, clearances, solver tolerance,
  weights and Mission lifecycle remain unchanged.
- Every candidate uses the same seven-state SQP, exact nonlinear trajectory
  adapter, exact swept wall proof, exact dynamic-obstacle proof and terminal
  successor proof as A--D.
- A shadow diagonal solve is never publishable.
- A numerical solution without both exact physical proofs is rejected.
- Existing production snapshots retain their fingerprint.

## Acceptance

- The diagonal schedule is sealed into each candidate fingerprint.
- Endpoint rows are algebraically identical to the existing stay-behind and
  pass-side rows.
- Intermediate rows contain both physical effective-progress and lateral
  coefficients.
- The frozen comparison enumerates both homotopies and bounded transition
  schedules and reports solver/proof outcomes.
- Focused tests, full package tests and `make autoware-build` pass.

## Non-scope

- No production obstacle policy change.
- No clearance, wall margin, weight, horizon, timeout or solver tuning.
- No new fallback, retry, lease, grace period or normal authority.
- No claim of physical infeasibility from local optimizer failure.
