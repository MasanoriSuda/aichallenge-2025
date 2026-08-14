# Tasklist

- [x] Analyze `20260815-015514` authority release timing.
- [x] Separate optimizer freshness from runtime physical validation in design.
- [x] Extend the core authority policy and regression tests.
- [x] Track per-path runtime validation state in the controller.
- [x] Renew the lease only after full execution revalidation.
- [x] Add configuration and transition diagnostics.
- [x] Run focused tests, package tests and build.
- [x] Record verification results.

## Verification results

- `make autoware-build`
  - 25 packages completed; `multi_purpose_mpc_ros` built successfully.
- Focused `*FrenetDpExecution*` GoogleTest run
  - 13 tests passed, 0 failed.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 1133 tests, 0 errors, 0 failures, 0 skipped.
  - `colcon test-result` also reported the pre-existing missing
    `build/joycon_contract_guard/package.xml` result-file warning; the selected
    package still completed all 25 test executables successfully.
- `git diff --check`
  - Passed.

Dynamic `make dev2` verification remains a user-run follow-up. Expected logs:

- startup: `runtime_lease<=0.20 s`;
- optimizer ownership: `source=fresh_optimizer`;
- temporary optimizer miss: `source=runtime_revalidation` while source age is
  greater than 0.50 s;
- authority release only after validation outage or an explicit hard fault,
  not solely because optimizer source age crossed 0.50 s.
