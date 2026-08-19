# Tasklist

- [x] Correlate the `20260819-100123` failure with solved-source authority logs.
- [x] Identify the wall-validation/continuous-handoff authority gap.
- [x] Add a pure connected bridge-admission policy.
- [x] Gate controller execution authority and warning suppression with that policy.
- [x] Add regression tests.
- [x] Build and test `multi_purpose_mpc_ros`.
- [x] Review the scoped changes.
- [x] Commit the scoped changes.

## Verification

- `make autoware-build`: passed, 25 packages completed.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed, 28/28 CTest targets.
- `colcon test-result`: 1351 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
