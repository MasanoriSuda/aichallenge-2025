# Validation

## Root-cause result

The residual five-state surface had three different meanings and could not be
removed as one undifferentiated block:

1. the left/right tactical pre-entry Gate A is live but commandless;
2. the old Overtake publisher and its async/retained lifecycle were rooted at
   functions with zero callers and were therefore unreachable but
   reconnectable normal authority;
3. Emergency output falsely named the five-state formulation despite running
   no solve.

This Slice removes only (2), corrects (3), and preserves (1) until a
prospective six-state Gate A obtains evidence and is promoted atomically.

## Failure-first evidence

`test_unreachable_five_state_overtake_owner_is_physically_deleted()` failed
before the implementation because the retired publisher and async selector
symbols still existed. After the deletion, the complete source-contract file
passes 49/49 tests and still proves that tactical branch evaluation cannot own
actuation.

## Static validation

- `git diff --check`: passed.
- Direct source-contract execution: 49/49 passed.
- `make autoware-build`: 25 packages completed successfully; only existing
  Python `setup.py install` deprecation warnings were emitted.
- Final Docker package test: 49/49 CTest targets and 1,864 tests passed in
  20.13 seconds. `colcon test-result` also reported a pre-existing stale
  `joycon_contract_guard/package.xml` result path, but summarized zero errors,
  zero failures and exited successfully.
- Static symbol search finds no `canonical_normal_control()`,
  `evaluate_overtake_async_shadow()`, retired Overtake async mailbox/selector,
  or retained five-state Overtake plan store.
- The live Gate A bundle is named `TacticalPreentrySelection`; it is not
  represented as a generic normal-command selector.
- The remaining `VelocityProgress5State` producers are limited to tactical
  pre-entry proof and generic formulation sealing; Emergency no longer claims
  that formulation.

## Dynamic validation

Bounded two-vehicle run: `output/20260825-182148`.

| Evidence | Domain 1 | Domain 2 |
|---|---:|---:|
| final execution contracts | 25 | 9 |
| certified normal six-state contracts | 15 | 5 |
| five-state final execution contracts | 0 | 0 |
| unresolved explicit Emergency contracts | 10 | 4 |
| retired async/publisher symbol traces | 0 | 0 |

Both domains continued normal publication as
`velocity-steering-progress-6state`. Missing proof failed closed as explicit
`emergency-override` with `formulation=unresolved`; it was not fabricated as a
five-state solve. The short run reached Follow and tactical Overtake candidate
evaluation, but did not enter ShiftOut/Pass. That is sufficient for the
behavior-neutral dead-owner deletion gate, not for replacing the live tactical
Gate A.

## Residual boundary

The only intentional five-state responsibility is the left/right tactical
pre-entry Gate A. Its replacement requires a separate Slice that first
produces a prospective six-state entry artifact, obtains dynamic acceptance,
then deletes the five-state Gate A in the same authority change. Parameter,
solver, horizon, clearance and timeout tuning remain prohibited before that
boundary is closed.
