# Requirements

## Goal

Complete the current MPCC single-authority migration without adding another
symptom-specific fallback.  The first Slice in this steering directory fixes
the six-state canonical continuity defect observed in
`output/20260826-050947` before any further Overtake promotion or parameter
tuning.

## Evidence boundary

- branch: `develop_july`
- baseline HEAD: `9198f79a309eb777230b6a3a93bda21dcf796668`
- dynamic run: `output/20260826-050947`
- primary incident: Domain 3 Cruise authority alternates between a retained
  six-state solution and `retained-proof-unavailable`, then reaches sustained
  QP rejection and Recovery.

## Required order

1. Identify the first invalid assumption in the retained-source causal join.
2. Add a failure-first regression test reproducing that assumption.
3. Correct the shared six-state contract; do not tune solver or vehicle
   parameters.
4. Verify Track/Cruise continuity dynamically.
5. Verify ShiftOut, direct Pass, Pass and Return acceptance evidence.
6. Promote direct Pass from the remaining five-state Gate A to six-state and
   delete the replaced lifecycle in the same Slice.
7. Physically remove remaining legacy/migration paths in Slice 6.
8. Run the integrated `make dev`, `make dev2`, `make dev3` quality gate.

## Constraints

- Preserve ROS topics, services, Domain layout, launch entry points and result
  schemas.
- Do not modify `aichallenge/result-summary.json` (pre-existing user change).
- Do not add a new fallback, lease, timeout, feature flag or special-case
  authority path.
- Do not tune OSQP, wall margins, speed, acceleration or steering parameters.
- A production promotion must delete the superseded normal authority in the
  same Slice.
- Dynamic evidence is required before an authority promotion.

## Definition of done for the continuity Slice

- A failure-first test demonstrates the invalid causal join.
- The corrected test and the complete package test pass.
- A moving dynamic run no longer alternates a valid normal six-state command
  with emergency braking solely because an asynchronous artifact is retained.
- Solver failure, wall rejection and Recovery remain fail-closed when their
  physical predicates are genuinely violated.
- The Slice is documented and committed independently.
