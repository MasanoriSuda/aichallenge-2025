# Requirements

## Objective

Determine whether the observation-only latest-state feedback QP can replace
direct production adoption of an asynchronous preparation artifact without
mixing control origins. Production promotion is conditional on dynamic
acceptance; failed promotion code must not remain.

## Evidence baseline

- Baseline commit: `8ae755cc`.
- Bounded A/B run: `output/20260829-041453`.
- d1 complete-proof recovery: 1,758 / 2,823 attempted feedback solves.
- Synchronous feedback maximum: 50.627 ms (d1), 135.235 ms (d2), therefore
  forbidden in the 25 ms control callback.

## Invariants

- One canonical seven-state MPCC normal authority remains.
- Preparation artifacts are numerical provenance only and never enter the
  certified-plan Store.
- Only a latest-state feedback artifact joined to its exact physical-wall
  certificate may enter the Store.
- Current-world, dynamic-obstacle and Follow proof remains mandatory before
  command authority.
- No solver tolerance, clearance, lease, timeout or fallback changes.
- Existing interface, topic, Domain and result contracts remain unchanged.

## Required validation

- Unit tests cover preparation-only and feedback-certified Store admission.
- Source-contract tests prove the old preparation-to-Store path is absent.
- Worker failure or delay cannot block the control callback or erase the last
  actually published certified artifact.
- `make dev2` shows bounded callback runtime and canonical production commands.

## Acceptance result

Rejected for production. The bounded run `output/20260829-044727` proved that
the worker boundary itself is non-blocking, but the feedback QP frequently
became dual-infeasible or reached maximum iterations. Accepted feedback
artifacts always passed exact physical-wall proof, so proof was not the cause.

The dominant failed rows were future seven-state box rows, especially stage-1
progress. The preparation is built at its captured local course-progress
origin. Feedback replaced only state zero and the serialized previous input,
while retaining future progress/lag/heading boxes and SQP linearizations from
the older origin. That is not an AS-RTI preparation around the predicted
feedback state.

No tolerance, wall clearance, lease, timeout or fallback change is permitted
as a response to this rejection.
