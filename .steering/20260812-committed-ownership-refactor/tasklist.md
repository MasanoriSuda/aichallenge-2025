# Tasklist

- [x] Inspect current ShiftOut/Pass ownership and ContactContinuation policy.
- [x] Extract shared committed-execution ownership guards.
- [x] Extract Pass geometry ownership resolution.
- [x] Keep existing ownership entry points behaviorally equivalent.
- [x] Add focused unit tests for extracted resolutions.
- [x] Build `multi_purpose_mpc_ros` in the development container.
- [x] Run package tests and record results.

## Definition of Done

- No parameter, ROS interface, or state-transition policy is changed.
- Existing and new ownership tests pass.
- `multi_purpose_mpc_ros` builds successfully in the project container.

## Verification results

- `git diff --check`: passed.
- Docker `colcon build`: 25 packages passed.
- Docker `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 tests passed.
- Build stderr contained only the existing `setuptools` deprecation warning.
- Dynamic `make dev2` was not run because this refactor intentionally leaves
  runtime behavior unchanged.
