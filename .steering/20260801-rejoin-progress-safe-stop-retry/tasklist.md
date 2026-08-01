# Task List

- [x] Confirm the P2 rejoin and P1/P2 mutual-stop timeline.
- [x] Define progress-aware bounded rejoin behavior.
- [x] Implement progress tracking and configuration loading.
- [x] Permit checked aggressive SafeStop retry during persistent solver fallback.
- [x] Add convergence, stall-timeout, and fail-safe unit tests.
- [x] Run package tests, build, and `git diff --check`.
- [x] Record dynamic verification points.

## Static Verification

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets succeeded.
- Test result: 759 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: succeeded.

`colcon test-result --verbose` also reported an unrelated stale
`build/joycon_contract_guard/package.xml` lookup warning, but returned success and the selected
package had no test errors or failures.

## Dynamic Verification

Run `make dev2` and reproduce the P1/P2 contact:

- P2 must remain in `LOW_SPEED_REJOIN` beyond five seconds while its normalized alignment error
  is materially improving;
- P2 must either reach `rejoin_complete` or stop/reassess if alignment stalls or a checked path
  gate becomes unsafe;
- a persistent normal-MPC fallback must not freeze an otherwise recoverable SafeStop retry;
- neither kart may remain at speed zero solely because the other stopped kart occupies its
  recovery corridor.
