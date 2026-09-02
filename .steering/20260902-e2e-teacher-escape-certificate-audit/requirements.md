# Requirements

## Objective

Determine why the current `speed_committed_teacher` can finish the certified
four-peer world yet becomes physically trapped in the deterministic mixed MPC
peer world.  The Slice is an offline root-cause audit; it does not modify
production authority, teacher policy or model artifacts.

## Frozen cases

- Success:
  `output/20260902-e2e-final-speed-committed-teacher-all-v2/d3`.
- Independent success:
  `output/20260902-e2e-final-speed-committed-validation-v1/d3`.
- Failure:
  `output/20260902-e2e-peer-speed-committed-teacher/d3`.

All cases used the same raw TinyLidarNet identity and
`speed_committed_teacher`.  Their different worlds and outcomes must remain
visible in the report.

## Questions

1. Is `front_distance < required_stop_distance` specific to the failure?
2. Does the teacher publish forward authority without an explicit physical
   escape certificate?
3. Can the existing scan, side commitment and selected gap prove vehicle-body
   clearance through a terminal successor, or do they prove only an
   instantaneous angular opening?
4. Would a longitudinal threshold hide the symptom while rejecting successful
   cornering states?

## Constraints

- Replay each stateful teacher from the first scan in timestamp order.
- Synchronize only the latest preceding fresh wheel-speed sample.
- Preserve the frozen base checkpoint and teacher parameters.
- Do not extract labels or train from the failed run.
- Do not introduce a timeout, clearance threshold, speed cap or runtime
  fallback in this Slice.
- Report structural proof absence separately from observed geometric failure;
  absence of a certificate is not itself proof of collision.

## Definition of Done

- A deterministic report compares the same teacher fields in both successful
  and failed bags.
- Unit tests cover causal speed synchronization and the audit classification.
- The report either identifies a discriminating invariant or explicitly
  rejects the current teacher representation as insufficient to provide one.
- The next implementation choice is recorded before any runtime change.
