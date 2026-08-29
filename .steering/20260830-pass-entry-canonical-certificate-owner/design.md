# Design

## Root cause

The production command and the tactical phase transition have different proof
owners:

1. The canonical seven-state pipeline creates an immutable execution artifact.
2. Exact wall proof, current-world opponent proof, actuation reachability, and
   Stop-contingency proof accept it.
3. The accepted command crosses the publisher and is recorded in the execution
   ledger.
4. At `ShiftOut -> Pass`, the tactical code resamples that same exact trajectory,
   clamps it toward a legacy lateral goal, and asks `OvertakeLineHorizon` to
   certify it again.
5. The approximate projection can request a wall clamp even while the canonical
   exact trajectory is currently executing safely, so the Mission is invalidated.

The failure observed as `DynamicMissionWait` is downstream. The first invalid
owner edge is the second certificate in step 4.

## Authority model after this Slice

- Canonical, matching, actually published ShiftOut artifact:
  - owns Pass-entry execution certification;
  - no legacy projection is run;
  - existing runtime hard-wall guard remains authoritative;
  - atomic intent admission keeps ShiftOut published until Pass proof joins.
- Canonical identity is expected but unavailable:
  - transition remains unavailable;
  - no fallback to DP/legacy evidence.
- No canonical published identity exists:
  - legacy/DP projection may still act as preflight for the migration path;
  - it is not promoted to canonical command authority.

## Implementation

Add a pure `PassEntryCertificatePolicy` resolver to
`v2x_overtake_core`. The controller calls it before constructing the legacy
projection. The policy has three explicit outcomes:

- `CanonicalPublishedExecution`
- `ProjectedExecutionPreflight`
- `Unavailable`

This makes the owner decision testable and prevents later edits from silently
restoring the duplicate evaluation.

## Non-goals

- Worker scheduling and latest-only queue improvements.
- Candidate-generation changes.
- Parameter tuning.
- Removing all remaining Mission geometry in this Slice.
