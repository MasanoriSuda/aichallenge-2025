# Tasklist

- [x] Inspect `20260815-010313` and compare it with `20260815-003734`.
- [x] Identify the shadow-evaluation/output ordering bug.
- [x] Identify prefix-first refresh starvation of complete-Mission fallback.
- [x] Publish the current locked-side shadow DP prefix.
- [x] Add prefix-to-complete refresh fallback.
- [x] Build a contracted-goal DP transition path for runtime wall contraction.
- [x] Keep DP authority after an atomic same-side Mission rebase.
- [x] Add focused unit tests.
- [x] Run package build/tests.
- [x] Record verification results.

## Verification

- `make autoware-build`
  - Passed: 25 packages built.
  - Existing colcon override and setuptools deprecation warnings only.
- `test_v2x_overtake_core --gtest_filter='*FrenetDpExecution*'`
  - Passed: 9/9 focused tests.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - Passed: 1102 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`
  - Passed.

## Dynamic acceptance checks

- `source=receding_prefix` is observed during an active ShiftOut/Pass.
- DP age does not monotonically grow while a feasible shadow prefix is shown.
- Runtime wall contraction retains an active reference aligned with the new
  goal.
- `Pass -> Return -> Idle` remains possible.
