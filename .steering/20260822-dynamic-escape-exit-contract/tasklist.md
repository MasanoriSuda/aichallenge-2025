# Tasklist

- [x] Correlate Dynamic Escape exit, predicted wall path, contact, and recovery.
- [x] Add retained Dynamic Escape execution snapshot.
- [x] Add short progress-contouring formulation lease.
- [x] Add active/exit wall admission and alternate-side replan trigger.
- [x] Add change-aware decision logs.
- [x] Add/update unit tests.
- [x] Build and run package tests.
- [x] Review diff and commit without generated/user-owned artifacts.

## Verification

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32 CTest
  targets passed. Final package-scoped result: 1436 tests, 0 errors, 0
  failures, 0 skipped.
- `colcon test-result --verbose` also reported an unrelated stale
  `build/joycon_contract_guard/package.xml` lookup, while its final test summary
  remained 1495/0/0/0.
