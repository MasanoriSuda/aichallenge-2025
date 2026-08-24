# Validation

## Static

- Source-contract regression was added first and failed because the dual branch derived the next
  generation from mutable `overtake_line_state_` inside the worker.
- `test_single_authority_source_contract.py`: 13 passed.
- `git diff --check`: passed.
- `make autoware-build`: 25 packages passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 tests passed,
  1,729 assertions/tests, zero error/failure/skip.
- Source-contract inspection also proves the replacement plan store is accepted before the
  canonical lifecycle identity is mutated, and that the adoption path cannot clear the old plan.

## Dynamic

### Failed implementation Gate

- Run: `output/20260824-113223`, Domain 1.
- Live Mission generation 0 required a prospective generation-1 artifact.
- Worker-private state mutation produced generation 2.
- Entry was rejected as `intent-generation-mismatch` after each speculative Mission freeze.

### Accepted regression Gate

- Run: `output/20260824-114633`, Domain 1.
- First accepted entry froze Mission generation 1 exactly once.
- `Idle -> ShiftOut` occurred immediately afterwards.
- First sampled ShiftOut command used the same stored five-state plan through
  `canonical-shiftout-retained`; no Overtake `async-pending` or legacy normal owner was inserted.
- Runtime replacement was not attempted because no complete replacement artifact became available.

## Separate follow-up evidence

- Episode 1: `ShiftOut -> FollowPrepare -> Recovery` because DynamicWait had no valid current-side
  prefix/lateral authority.
- Episode 2: retained current-world proof rejected `current origin rejected: discontinuous`, then
  `initial-corridor-violation` and certificate expiry.
- These failures occur after the corrected atomic entry and remain visible for the next root-cause
  Slice. This Slice does not tune them or add another fallback.
