# Results: Pass terminal Stop owner audit

## Observed chain

`output/20260831-081212/d2` admitted Pass atomically at decision 3379.  The
frozen Mission execution source later became stale, current-side replan was
unavailable, decision 3418 lost its recursive terminal Stop proof, Emergency
Stop interrupted normal authority, and actual footprint wall contact appeared
only at decision 3424.

## Earliest violated invariant

The semantic seven-state adapter applied both the immutable initial-state
equality and the conservative future planning box to stage zero.  At decision
3418 the measured lateral state was `0.943174 m`, beyond the approximate upper
support `0.911453 m`, although exact occupancy-grid proof reported no current
contact.  Every candidate was therefore rejected before any control could be
optimized.  The Stop lattice carried a producer-local rebase for the same
contradiction, confirming duplicate ownership.

## Same-snapshot comparison after repair

At decision 3418 every candidate now reaches solve/proof, but all normal and
Stop trajectories hit the exact wall.  The maximum-braking Stop first contacts
at stage 2; common Pass candidates contact around stage 5.  This snapshot is
already unavoidable and cannot identify the upstream defect.

At decision 3396:

| arm | result |
| --- | --- |
| A persistent Pass | rejected by dynamic-obstacle SQP |
| B stateless current side | rejected |
| B stateless opposite side | exact-certified Bundle |
| C production current-world positive side | exact-certified mid-diagonal Bundle |
| C production current-world negative side | exact-certified direct-side Bundle |

The classification is `A fails, B succeeds: Mission lifecycle defect` at the
architecture level, while the production correction is the duplicate
stage-zero constraint that prevented the already-existing current-world
population and sibling bank from supplying those Bundles live.  No clearance,
solver tolerance, timing, lease, grace, fallback or authority rule changed.

## Changes

- Stage-zero legacy state bounds are now owned centrally by the exact measured
  initial-state equality.
- Future planning boxes and exact wall/dynamic/terminal proof are unchanged.
- The Stop producer's local stage-zero rebase is removed.
- Regression tests prove stage zero is exact while future bounds remain intact.
- The worker test uses an explicit startup handshake and always releases its
  blocked job on an assertion path; production code is unchanged.

## Dynamic acceptance target

Bounded `make dev3` validation was recorded in `output/20260831-085329`.
The repaired adapter no longer rejected the relevant ShiftOut/Pass requests as
`initial-state-outside-bounds`; both sides reached the seven-state solve and
exact physical proof.  This accepts the stage-zero ownership repair.

The run also exposed a separate scheduling/lifecycle defect.  At D2 decision
1506 the frozen offline comparison found an exact-certified opposite-side
Bundle while the selected side failed wall proof (`A fails, B succeeds`).  Live
execution did not receive that Bundle before wall contact: the active Overtake
bank still held decision 1493 because the dual evaluator withheld the completed
side until its slower sibling also finished.  This is not addressed by this
Slice and is the input to the next structural Slice.

## Validation

- `make autoware-build`: 25 packages succeeded.
- CTest from the canonical workspace build directory
  (`/aichallenge/workspace/build/multi_purpose_mpc_ros`): 59/59 passed.
- Bounded dynamic validation: `output/20260831-085329`.
- The similarly named `/aichallenge/build/multi_purpose_mpc_ros` tree contained
  stale August 29 artifacts and is not a valid test source.
