# Task list

- [x] Confirm the latest failure boundary and preserve the runtime lease.
- [x] Add a deterministic longitudinal-profile selector to `v2x_overtake_core`.
- [x] Allow `V2XGapPlanner` path-time prediction to accept a bounded ego-speed override.
- [x] Evaluate and select target-aware Frenet DP corridors across timing profiles.
- [x] Propagate selected closing speed into Mission candidate rollouts and diagnostics.
- [x] Add matching local/cloud parameters and startup logging.
- [x] Add unit tests.
- [x] Run build/tests and record results.

## Verification

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets passed.
- Aggregate `colcon test-result --verbose`: 1136 tests, 0 errors, 0 failures.
- `colcon test-result` also reported a pre-existing stale
  `build/joycon_contract_guard/package.xml` lookup warning; it did not affect the selected
  package or test result.
- Host `git clang-format` was unavailable. `git diff --check` passed.
- Dynamic `make dev2` effect verification remains for the next run.
