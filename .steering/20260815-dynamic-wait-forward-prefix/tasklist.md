# Tasklist

- [x] Inspect the latest run and identify the dominant wait path.
- [x] Inspect current Mission ownership, branch replacement and speed policy.
- [x] Add the bounded forward-prefix policy and tests.
- [x] Integrate fresh current-side prefix validation in DynamicMissionWait.
- [x] Preserve immediate atomic alternate replacement priority.
- [x] Add transition diagnostics.
- [x] Run focused tests, package tests, build and diff checks.
- [x] Record verification results.

## Verification

- `make autoware-build`: passed (25 packages; only existing setuptools deprecation warnings).
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: 1142 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
- Dynamic driving verification remains for `make dev2`; compare DynamicMissionWait duration,
  `planning_unavailable` time, minimum speed, alternate replacement latency, wall recovery and
  contact counts against `output/20260815-071257`.

The unrelated missing `build/joycon_contract_guard/package.xml` diagnostic from
`colcon test-result` remains, but it did not affect the zero-failure package result.
