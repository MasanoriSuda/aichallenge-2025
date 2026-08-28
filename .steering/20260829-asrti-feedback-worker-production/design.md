# Design

## Pipeline

```text
current world snapshot
  -> latest-only preparation worker
  -> immutable final QP preparation + observation-only projection plan
  -> current control-origin state join
  -> latest-only feedback worker
  -> latest-state QP + exact nonlinear/wall proof
  -> certified-plan Store
  -> unchanged current-world retained proof
  -> canonical command / publisher
```

The projection plan exists only to reconstruct the current seven-state Frenet
state against the same course/identity.  It cannot be stored or published.

## Atomic deletion

The preparation worker must call the existing pipeline with no certified-plan
Store.  The only Store replacement in this pipeline moves to the feedback
worker after its exact physical proof succeeds.

## Scheduling

Both preparation and feedback use independent `LatestOnlyWorker` instances.
Each has one running and one replaceable pending job.  Control callback work is
limited to immutable snapshot construction, mailbox exchange and final
current-world proof; no QP solve executes in the callback.

## Failure behavior

- Preparation failure: no feedback submission; last executed certified plan
  remains governed by existing current-world proof.
- Feedback failure: no candidate Store replacement.
- Stale identity/geometry/intent: discard before feedback submission.
- Worker exception: contained by `LatestOnlyWorker`; no emergency owner is
  introduced.

## Non-goals

- No candidate-generation or parameter tuning.
- No Overtake-specific tactical change in this Slice.
- No new grace period, lease, timeout or fallback.

## Dynamic acceptance finding

The proposed pipeline is architecturally non-blocking but numerically invalid:

```text
captured local progress origin + old future state tubes/linearizations
  + latest state zero expressed at a later control origin
  -> mixed-origin QP
  -> stage-1 progress box infeasibility / maximum iterations
  -> no certified successor
  -> retained horizon exhaustion
  -> Emergency Stop
```

`control_prediction_origin_sec` was a valid latency-compensated timestamp and
the vehicle model was projected to that origin when the source request was
built. The defect begins later: after asynchronous preparation completes, the
feedback join advances to another control origin but does not rebuild or
predict the remaining stage problem for that origin.

The regression test
`ArchitectureEscapeHatchClassifiesOldOriginFeedbackAsProblemRebuildDefect`
proves both sides of the classification:

- old prepared final QP + later x0: rejected;
- same physical state rebased into a current-world problem whose references,
  bounds and linearizations are rebuilt together: solved.

This excludes physical infeasibility and parameter strictness for the frozen
case. Production wiring from this rejected design is removed atomically. A
future Slice must prepare around the predicted feedback/control origin (AS-RTI)
or use a complete current-world rebuild; it may not relabel an old problem by
changing only its timestamp and x0.
