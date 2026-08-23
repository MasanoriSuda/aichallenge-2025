# Canonical normal async transport audit

## Observed architecture before this Slice

1. The accepted Follow producer already used an immutable problem snapshot, a
   latest-only worker, a typed mailbox and live current-world revalidation.
2. The mailbox identity did not carry `ControlIntent`; all three validators
   hard-coded `Follow`.
3. Overtake canonical shadow evaluation still runs synchronously in the 40 Hz
   callback and production converts its five-state result back into the legacy
   execution path.
4. Copying the Follow mailbox for Overtake would create a second authority
   boundary with behavior-specific validation and make later legacy deletion
   harder to prove.

## Failure-first evidence

The new ShiftOut snapshot test was added before the implementation. It failed
because `validate_snapshot_context()` returned `IntentMismatch` for an exact,
sealed ShiftOut context. This demonstrates that the missing reusable boundary,
not a solver parameter, blocked the next migration Slice.

## Root cause classification

- Root cause: the asynchronous result identity omitted the exact normal-control
  intent and the transport inferred Follow semantics internally.
- Contributor: implementation and public names were Follow-specific although
  the mailbox itself contains no Follow control policy.
- Masking logic: none added. Existing Follow behavior remained the sole live
  producer and consumer.
- Detection gap: there was no test proving that every supported canonical
  normal intent can cross the immutable transport without re-derivation.

## Implemented boundary

`ResultIdentity` now seals the exact `ControlIntent`. Snapshot, worker payload
and current-context validation compare that same intent and reject unsupported
or cross-intent data. Target identity remains mandatory only for intents whose
execution contract requires a target.

The canonical namespace is now `canonical_normal_async`. The old
`follow_canonical_async` namespace remains only as a source-compatibility alias;
there is one implementation and one mailbox contract.

## Authority audit

- No new producer is connected.
- No publisher or selector input changed.
- No Overtake command is made executable.
- Follow remains the only live user and submits the same sealed Follow context.
- No fallback, timeout, retry, lease, solver option or configuration was added.

This Slice therefore removes a structural blocker without claiming production
Overtake coverage or authority.
