# Requirements

## Objective

Close the remaining runtime Overtake authority gap: a same-side or cross-side Mission replacement
must never change Mission generation, side or phase unless the exact five-state MPCC execution
artifact selected with that Mission is ready for canonical production authority.

## Evidence baseline

- Source baseline: `bf6361b`.
- Dynamic baseline: `output/20260824-110945/d1/autoware.log`.
- Initial `Idle -> ShiftOut` adoption is accepted and starts with
  `canonical-shiftout-retained`.
- The later DynamicWait cross-side replacement freezes generation 2 and changes
  `FollowPrepare -> ShiftOut`, but its first control cycle is `async-pending` Emergency.
- Subsequent commands alternate between retained authority and
  `initial-corridor-violation`, `invalid-progress-evolution` and progress-discontinuity rejects.

## Invariants

1. Tactical left/right selection produces one typed artifact containing both the selected Mission
   and its immutable canonical execution plan.
2. Mission identity and plan identity agree on intent, prospective Mission generation, target,
   target observation generation and side.
3. Runtime replacement exposes a new Mission generation, side or phase only after the matching
   canonical plan has been accepted; a rejected plan restores the prior Mission and plan together.
4. A Mission-only result is not production-authority evidence and cannot replace active execution.
5. Current-world wall, target and corridor proof remains mandatory after adoption.
6. No new grace, timeout, lease, fallback or parameter tuning is introduced.
7. Existing user changes and generated output remain untouched.

## Definition of done

- Deterministic tests reject Mission-only and identity-mismatched runtime replacements.
- Same-side/cross-side outputs cannot report ready without a matching canonical plan.
- Runtime replacement is transactional and does not expose a phase with missing canonical
  authority.
- Package build/tests and source-contract tests pass.
- `make dev2` demonstrates either an atomic runtime replacement with no entry `async-pending`
  Emergency, or no runtime replacement if no complete artifact is available.
- The Slice documents any later, separate progress/corridor defect instead of masking it.
