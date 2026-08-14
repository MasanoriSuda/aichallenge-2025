# Tasklist

- [x] Analyze `20260815-012736` against `20260815-010313`.
- [x] Identify legacy Pass-extension authority after DP refresh.
- [x] Identify discarded Return-preflight reference.
- [x] Add the bounded DP Pass authority policy and tests.
- [x] Bypass only legacy same-side/longitudinal refresh while DP owns Pass.
- [x] Persist and execute the Return-preflight reference.
- [x] Add Return reference regression tests.
- [x] Run focused tests, full package tests and build.
- [x] Record verification results.

## Verification results

- `make autoware-build`
  - 25 packages completed; `multi_purpose_mpc_ros` built successfully.
- Focused `*FrenetDpExecution*` GoogleTest run in the development container
  - 12 tests passed, 0 failed.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 1132 tests, 0 errors, 0 failures, 0 skipped.
  - `colcon test-result` also reported a pre-existing missing
    `build/joycon_contract_guard/package.xml` result-file warning; it did not
    change the successful package-test result.
- `git diff --check`
  - Passed.

Dynamic `make dev2` verification remains a user-run follow-up. Confirm that a
fresh DP prefix logs `DP Pass authority retained`, that Return logs
`Return preflight reference committed`, and that the mission reaches
`Pass -> Return -> Idle` without the former legacy same-side veto.
