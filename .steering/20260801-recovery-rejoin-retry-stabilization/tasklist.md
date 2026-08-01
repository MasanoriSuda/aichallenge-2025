# Task List

- [x] Confirm clean target scope and preserve unrelated user changes.
- [x] Review current Rejoin progress tracker and aggressive retry transition.
- [x] Add Rejoin lateral regression detection and bounded reassessment.
- [x] Add snapshot-gated aggressive retry with change-based re-arm.
- [x] Remove sticky Forward fallback preference.
- [x] Add focused pure-core and supervisor tests.
- [x] Run package tests.
- [x] Run `make autoware-build` equivalent Docker command.
- [x] Record verification results and remaining dynamic-test risk.

## Verification

- `docker compose run -T --rm --no-deps autoware-build`
  - 25 packages finished; build successful.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 25/25 test targets passed in the full package run.
- Final focused rerun: `test_stuck_recovery_core`
  - 116/116 tests passed after the final candidate-steering snapshot change.
- `git diff --check`
  - Passed.

## Dynamic check remaining

- Run `make dev2` and reproduce collision recovery.
- Confirm one `aggressive_retry` for an unchanged failed snapshot followed by throttled
  `aggressive_retry_awaiting_change`, rather than hundreds of cycles.
- Confirm a Rejoin that captures `|e_y| <= 0.5 m` and then exceeds `0.7 m` transitions with
  `rejoin_regressed` before multi-metre overshoot.
- Confirm a genuinely changed candidate/contact/V2X snapshot re-arms Recovery.
