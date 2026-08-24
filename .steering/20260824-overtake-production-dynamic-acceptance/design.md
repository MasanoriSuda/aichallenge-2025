# Design

## Strategy

This is an acceptance and root-cause Slice, not a behavior-development Slice. The production path
established by `174b268` is frozen while dynamic evidence is collected.

## Authority join

For every candidate decision, join the following existing telemetry:

```text
observation/target generation
-> canonical intent and Mission generation
-> async problem fingerprint
-> solved/certified solution identity
-> immutable plan and cursor
-> current-world proof decision
-> exact actuation
-> final published authority
```

The join must be inspected separately for `ShiftOut`, `Pass`, `Return`, `DynamicWait`, and
`DynamicEscape`. A zero count does not establish correctness.

## Failure classification

| Class | Meaning |
|---|---|
| Root | The first producer or identity contract that becomes invalid |
| Contributor | Timing or geometry condition that raises failure probability |
| Mask | Hold, fallback, Recovery or phase churn that changes the visible symptom |
| Detection gap | Missing provenance at the earliest uncertain boundary |
| Recovery | Separate safety response after normal authority failed |

Solver failure, wall rejection, Emergency and Recovery are outcomes until the upstream artifact and
certificate are proven consistent.

## Pre-fix hypotheses

1. **Phase coverage gap**: the accepted Gate exercised only `ShiftOut`; no defect can be inferred for
   `Pass`/`Return` until they are observed.
2. **Physical infeasibility**: current-side ShiftOut may correctly fail the exact wall proof and enter
   Recovery. Falsifier: a wall-feasible opposite/current candidate exists with a complete canonical
   selection but is discarded before publication.
3. **Authority lifecycle gap**: a phase or DynamicWait transition may invalidate the async plan family
   before its replacement becomes selectable. Falsifier: matching current-world plan identity remains
   complete through the transition.
4. **Runtime budget contribution**: worker/callback latency may make otherwise valid evidence stale.
   Falsifier: result age and callback time remain within contract at the first failure.

## Change gate

No production source is changed merely because a run fails. A later implementation is allowed only
when the run identifies one earliest violated invariant, a deterministic pre-fix replay/test, the
producer to repair and the obsolete branch/mask to delete.

## Rollback

The rollback commit is `174b268`. Any implementation produced from this acceptance Slice must be a
separate commit and may not modify the user-owned result summary.
