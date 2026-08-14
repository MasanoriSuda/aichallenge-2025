# Task list

- [x] Correlate wall preplan warnings with the latest Pass failures.
- [x] Add the deterministic Pass-entry wall-gate resolver and tests.
- [x] Add bounded gate state and reset handling.
- [x] Hold a physically validated current-side lateral prefix during the gate.
- [x] Preserve speed only under current/predicted physical separation.
- [x] Make SafeSeparation entry idempotent.
- [x] Run focused tests, package tests and build.
- [x] Record verification results and dynamic acceptance criteria.

## Verification

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets passed.
- Aggregate `colcon test-result --verbose`: 1138 tests, 0 errors, 0 failures.
- `git diff --check`: passed.
- `colcon test-result` reported the pre-existing stale
  `build/joycon_contract_guard/package.xml` lookup warning; it did not affect the selected
  package or test result.
- Dynamic `make dev2` verification remains for the next run.
