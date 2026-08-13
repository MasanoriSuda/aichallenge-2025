# Task list

- [x] Record requirements and design.
- [x] Add post-validation convergence and highest-feasible-speed repair.
- [x] Release obsolete opponent bounds after confirmed physical body clear.
- [x] Add bounded diagnostics.
- [x] Build and run unit tests.
- [x] Record static verification results.
- [ ] Dynamic verification with `make dev2` (user-run).

## Static verification results

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets passed.
- `colcon test-result --verbose`: 1087 tests, 0 errors, 0 failures, 0 skipped. An unrelated stale `build/joycon_contract_guard/package.xml` lookup warning remains in the aggregate result scan.
- `git diff --check`: passed.
