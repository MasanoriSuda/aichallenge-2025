# Follow asynchronous canonical producer audit

## Confirmed observations

1. `get_control()` calls `evaluate_follow_shadow()` synchronously before it enters the existing
   normal/recovery solving branch.
2. `evaluate_follow_shadow()` builds, solves, validates, canonicalizes, stores and revalidates in the
   same callback.
3. The dynamic retained gate accepted 5 of its first 8 fresh-miss attempts and later accepted one
   additional command, so current-world execution does not require a same-cycle fresh solve.
4. The same run contains Follow solves at 4000 iterations and 20--27 ms. Around decision 4426 the
   enclosing callback reached 50.746 ms and 11/36 cycles exceeded the 25 ms period.
5. `LatestOnlyWorker` already supplies one-running/one-replaceable-pending semantics, exception
   containment, result sequencing and context-epoch rejection for tactical work.
6. `CanonicalExecutionPlanStore` is already thread-safe and rejects stale plan IDs.

## Cause chain

```text
Follow shadow eligible
-> five-state Follow QP solved synchronously in get_control
-> result remains shadow-only
-> get_control continues into legacy/recovery production solve
-> both computations consume one 25 ms callback budget
-> hard Follow QP reaches maximum iterations
-> callback deadline overruns although retained current-world execution exists
```

The expired retained plan seen later is a correct fail-closed outcome after a long non-Follow or
fresh-unavailable interval. Extending its certificate by age would be unsafe. The architectural
problem is requiring a fresh solve in the command thread, not the exact expiry reason.

## Hypothesis to validate

An immutable latest-only Follow producer plus current-world retained execution will preserve
canonical semantics while removing the double-solve deadline cost. This must be falsified if
snapshot construction itself, worker contention or result revalidation creates equivalent overruns.

## Rejected alternatives

- OSQP iteration/tolerance tuning before ownership separation.
- Sampling the synchronous shadow at an arbitrary lower frequency.
- Executing the last command by age without current-world proof.
- Publishing worker actuation directly from its old observation epoch.
- Adding another legacy/scalar fallback when the worker has no result.

## Slice A verification

The Follow solver context, warm-start identity/epoch and canonical plan store now share one
`FollowCanonicalLifecycle`. Short-lived tactical snapshots reference the same lifecycle rather than
constructing divergent state. Runtime solving remains synchronous in this sub-slice, so the change
isolates ownership without changing command semantics.

- `make autoware-build`: 25 packages successful.
- `ctest --test-dir /aichallenge/build/multi_purpose_mpc_ros --output-on-failure`: 38/38 passed.
- No normal authority, fallback, feature flag or parameter was added.

## Slice B verification

`follow_canonical_async` now defines a typed worker result and mailbox independent from the ROS node.
It rejects invalid plan payloads, internal plan/provenance mismatches, old context epochs, sequence
rollback and results that were never submitted. Rejected publication leaves the previously published
result unchanged; the mailbox cannot mutate the canonical execution-plan store.

- Formal `make autoware-build`: 25 packages successful.
- Focused CTest from `/aichallenge/workspace/build`: 3/3 passed.
- Full package CTest from `/aichallenge/workspace/build`: 39/39 passed.
- The older `/aichallenge/build` tree is not accepted as current evidence.
