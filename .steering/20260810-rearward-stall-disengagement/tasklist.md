# Task list

- [x] Confirm the measured local-distance failure path.
- [x] Add core policy types and resolver behavior.
- [x] Integrate state, configuration and controller transitions.
- [x] Add unit tests and operator-facing logs.
- [x] Run formatting/diff checks, package tests and build.
- [x] Record verification results.

## Verification

- `git diff --check`: passed.
- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 25 test targets passed.
- `colcon test-result --verbose`: 979 tests, 0 errors, 0 failures, 0 skipped.
- The test-result aggregator still reports the pre-existing stale
  `build/joycon_contract_guard/package.xml` warning; it does not belong to this
  package and did not change the successful package result.
