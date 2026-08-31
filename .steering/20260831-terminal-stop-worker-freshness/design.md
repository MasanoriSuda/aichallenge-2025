# Design

## Root cause

Two responsibilities were mixed in one production worker:

1. compute the latest seven-state terminal Stop successor needed by normal
   authority;
2. enumerate a wide steering-rate control lattice for architecture research.

The direct solve normally required about 45--70 ms, but a failed direct solve
entered the broad lattice and occupied the single worker for up to about
1.2 seconds.  `submit_latest_cancelable` also made every in-flight result
obsolete when each newly published artifact arrived.  Exact artifact identity
at consumer time therefore guaranteed that an asynchronous result was either
cancelled or stale.

## Production contract

```text
published ShiftOut/Pass artifact
  -> latest-only non-cancelling worker
       -> one direct seven-state maximum-braking Stop solve
       -> exact trajectory
       -> exact wall proof
       -> timed dynamic proof
       -> immutable CertifiedPlan
  -> same tactical scope check
  -> current-world retained revalidation
  -> canonical normal selection only if joined
```

The worker finishes its current bounded solve and coalesces pending work to the
newest observation.  A completed result is chronology-ordered by control
decision ID.  A certified plan may remain observable while target, Mission
generation, side and intent are unchanged; sequence and observation epoch are
not tactical state.  Every production use still performs the unchanged
current-world join.

## Audit-only population

The broad steering-rate lattice remains behind
`EvaluationMode::DirectThenControlLattice` for snapshot A/B/C/D comparison.
Production explicitly selects `DirectSevenStateOnly`.  This is a responsibility
split, not a new fallback or a relaxed certificate.

## Residual failure boundary

If the latest direct Stop is physically blocked by the opponent or wall, the
worker must report that fact.  It must not keep the vehicle moving through a
lease or tune the proof.  Such a failure means the upstream normal trajectory
was allowed to enter a no-escape state and belongs to a new admission/
continuation Slice.
