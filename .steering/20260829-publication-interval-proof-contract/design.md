# Design audit

## Observed phenomenon

Expected: if the command which can actually be placed on the wire for the next
25 ms is physically clear and has a certified Stop successor, retained normal
authority may publish that one command while a fresh solve is pending.

Actual: `CurrentStagePrefix` requires the complete remainder of the current
solver stage.  The frozen artifacts remain in stage 0 for hundreds of
milliseconds.  A lateral-bound departure later in that stage therefore erases
an otherwise clear next publication command.

## First violated invariant and producer

The first violation is a time-authority contract mismatch in
`mpcc_rate_resolved_physical_adapter::build_continuation()`: solver
discretization owns the minimum physical proof horizon.  The producer records
only `current_stage_sample_count`, and partial proof is possible only when the
failure occurs after that whole stage.

The retained static-wall and dynamic-obstacle validators repeat the same
assumption by truncating to `current_stage_remaining_sec`.

## Causal chain

```text
long solver stage remains active
-> next 25 ms command is physically clear
-> continuation crosses lateral bounds later in the same stage
-> no CurrentStagePrefix can be formed
-> terminal Stop proof is not attempted
-> retained normal authority disappears
-> Emergency Stop correctly owns the wire
-> stopped vehicle diverges from the old artifact and later enters Recovery
```

Root cause: proof horizon is coupled to solver-stage duration.

Contributor: stage 0 may be much longer than the 25 ms publisher period.

Mask/recovery: Emergency Stop is correct fail-closed behavior; it makes the
upstream proof loss visible as deceleration but is not the root cause.

Detection gap: the trace called the scope `current-stage-prefix`, hiding that
the code comment and production contract already promise one publisher
interval.

## Competing hypotheses

### H1: publisher/solver proof granularity mismatch — confirmed

Support: decisions 1464 and 2928 retain valid progress, steering and velocity
joins, then reject only as `invalid-lateral-bounds` within stage 0.  Source and
history show partial proof is gated by the complete current-stage sample count.

Falsifier: a deterministic replay in which the bound is crossed after 25 ms
but before stage end also rejects the exact 25 ms prefix.

### H2: current command is already physically infeasible — not supported

Falsifier: make the same deterministic scene cross the bound within 25 ms.  It
must remain rejected; dynamic replay must also show whether the frozen runtime
events are in this class.

### H3: wall or opponent proof causes the first loss — refuted for frozen events

The continuation is rejected before a pose path exists, so wall and dynamic
proof fields remain invalid/unattempted.

## Chosen correction

- Replace `CurrentStagePrefix` with `PublisherIntervalPrefix` throughout the
  physical, wall, dynamic and production proof vocabulary.
- Split nonlinear rollout sampling exactly at the publisher boundary.
- If the full suffix fails only after that boundary, retain exactly the
  publisher interval and its endpoint state.
- Evaluate partial wall and dynamic proofs over the same interval.
- Continue to require the separately certified Stop suffix.

This removes the incorrect current-stage partial-authority branch rather than
adding another fallback or timeout.

## Alternatives

1. Reduce solver stage duration to 25 ms: rejected because it changes the
   optimization problem and computation cost to repair an authority contract.
2. Ignore the later lateral violation: rejected because it would publish
   without a bounded successor.
3. Emergency Stop on every suffix defect: current behavior, safe but loses a
   physically certified publication opportunity and causes repeated races to
   stop.

## Implementation gate

- Root files: physical adapter and retained revalidation.
- Consumer files: production adapter, telemetry and focused tests.
- New production authorities/configuration: zero.
- Deleted semantic branch: remaining-current-stage partial authority.
- Remaining legacy normal authority: zero in this slice's execution path.
- Rollback commit: `8dc45378`.

## Unknowns

The frozen log does not contain the exact rejected sample time.  The new
deterministic regression proves the contract, and the next dynamic run must
show `publisher-interval-prefix` followed by a certified terminal Stop at the
first comparable authority loss.
