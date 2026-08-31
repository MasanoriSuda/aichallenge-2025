# Findings: stateless ShiftOut corridor authority

## Observed chronology

Frozen input: `output/20260831-162051/d1/autoware.log`.

- a sibling Bundle changed ShiftOut side `+1 -> -1` and crossed the publisher;
- the published current-world trajectory aligned on side `-1`;
- the six-state bridge recovered onto artifact `2182` and DP execution became
  active on side `-1`;
- a later legacy Mission/gap search rejected all candidates;
- despite the active certified trajectory, Behavior set
  `overtake_execution_corridor_blocked` and entered Recovery with
  `live overtake corridor unavailable`.

## Root cause

The gap planner's entry-candidate feasibility and the exact publisher-bound
trajectory both claimed to be the live corridor authority.  Candidate
regeneration could therefore revoke a different trajectory that had already
passed immutable identity, current-world proof and canonical publication.

## Change

- introduced one live predicate for the publisher-bound stateless source;
- reused it for committed ShiftOut Behavior ownership;
- made later gap-planner failure diagnostic-only while that source owns active
  ShiftOut/Pass execution;
- retained wall, target, emergency, continuity and solver aborts unchanged.

No timing hold, fallback, solver or clearance change was added.

## Validation

- source contract: `97 passed`;
- `test_v2x_overtake_core`: passed;
- `make autoware-build`: 25 packages successful;
- dynamic run: `output/20260831-163404/d1/autoware.log`.

The run adopted a sibling at line 1110.  Gap-planner failures were explicitly
reported as diagnostic-only at lines 1133--1200, and no
`live overtake corridor unavailable` transition occurred.  This satisfies the
Slice acceptance criterion.

## Next exposed defect

The same episode remained in ShiftOut until the published source disappeared,
then entered Recovery with `locked target stale or lost`.  That later failure
is not evidence against this repair.  It requires a separate audit of:

- ShiftOut completion progress after side adoption;
- why the exact published source was superseded at line 1234;
- whether target observation was genuinely lost or merely dropped by a
  lifecycle/identity boundary.
