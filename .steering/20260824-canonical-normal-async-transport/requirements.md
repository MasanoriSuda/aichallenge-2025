# Requirements

## Objective

Extract the accepted Follow latest-only result transport into one intent-aware
canonical-normal transport that can carry Overtake plans without weakening
identity checks.

## Root cause addressed

The current transport hard-codes `Follow` in snapshot, payload and current
identity validation. Reusing it for Overtake without an explicit intent field
would either reject every Overtake plan or require a second copied mailbox,
preserving two subtly different authority boundaries.

## Scope

- Preserve Follow production behavior.
- Add exact `ControlIntent` to worker result identity.
- Admit only canonical normal intents supported by the execution contract.
- Require snapshot, result plan and current context to match the exact intent.
- Keep the old Follow include/namespace as a source-compatible alias during
  the migration; record its deletion in Slice 6.

## Non-scope

- No Overtake worker connection or authority promotion.
- No solver, wall, vehicle or behavior parameter changes.
- No timeout, lease, retry, circuit breaker or fallback changes.

## Acceptance

- Failure-first test proves the old Follow-only validator rejects ShiftOut.
- Track/Cruise/Follow/ShiftOut/Pass/Return exact identities are accepted.
- Stop/Unknown and cross-intent payload/current context are rejected.
- Existing Follow mailbox tests, package tests and build pass.
