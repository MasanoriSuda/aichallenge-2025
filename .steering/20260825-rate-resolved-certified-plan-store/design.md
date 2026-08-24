# Design

## Evidence boundary

```text
six-state solve result
  + immutable ExecutionArtifact
  + exact physical wall Result(Accepted)
  + identical full snapshot identity
        -> CertifiedPlan
        -> monotonic CertifiedPlanStore
```

`CertifiedPlan` is deliberately not the five-state
`CanonicalExecutionPlan`.  Steering angle is a state and steering rate is the
input, so converting it into a curvature-input artifact would reintroduce the
formulation split this migration is removing.

The store is written inside the existing serialized latest-only worker after
physical evaluation.  A callback may snapshot it without blocking on solve or
wall proof.  Replacement is all-or-nothing and keyed by the artifact sequence;
failure never clears the prior certified plan.

## Deferred boundary

The original proof belongs to its captured pose and course-frame window.  It
does not by itself authorize later execution.  A following Slice must resolve
the artifact cursor and prove the current pose/connector/current semantics
against the remaining immutable horizon before any production promotion.
