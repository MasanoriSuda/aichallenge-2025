# Task list

- [x] Record requirements and design.
- [x] Add execution-bound and Pass-release core policies with tests.
- [x] Preserve hard and soft sample constraints separately in the controller.
- [x] Apply predicted-overlap confirmation to body-clear release.
- [x] Add failure-layer diagnostics.
- [x] Build and run package tests.
- [ ] Dynamic verification with `make dev2` (user-run).

## Verification

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: passed (25/25 test targets).
- Aggregate result: 1090 tests, 0 errors, 0 failures.
- Existing stale `build/joycon_contract_guard/package.xml` warning remains outside this change.
