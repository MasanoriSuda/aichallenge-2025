# Requirements

## Objective

Capture the first exact seven-state Track, Cruise, Follow or Rejoin solver
failure through the existing immutable architecture snapshot boundary.  The
recorder currently discards every non-Overtake intent, which prevents the
known Cruise-to-Follow authority hole from being classified without guessing
from aggregate logs.

## Frozen observation

Run: `output/20260828-170636`, Domain 1.

- decisions 4070--4105 proposed Follow while retaining certified Cruise;
- the candidate store still contained Cruise and therefore reported
  `intent-mismatch` when evaluated as Follow;
- fresh Follow sequence 3461 reached 4000 iterations and was rejected at the
  stage-12 velocity state upper bound;
- decision 4106 then lost the previous Cruise proof with
  `progress-lift-rejected` and emitted the existing Emergency command;
- no architecture snapshot was written because `record_failure()` returned
  `NotOvertake` for Follow.

## Constraints

- Observation only: do not change production authority, problem construction,
  solver settings, clearance, lease, fallback or command output.
- Keep interaction replay completeness separate from exact-QP replay
  completeness. A Follow snapshot may be exact-QP replayable even when the
  Overtake A--D ManeuverBundle comparison is not applicable.
- At most one snapshot per intent/pipeline/outcome remains the process bound.

## Acceptance

- A Follow solve rejection is written and exact-QP replayable.
- Unknown, Hold and Stop remain ineligible because they are not canonical
  seven-state normal intents.
- Existing Overtake interaction snapshots and comparison loading are unchanged.
- Build and all package tests pass.
