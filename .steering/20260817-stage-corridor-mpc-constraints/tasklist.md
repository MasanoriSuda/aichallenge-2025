# Tasklist

- [x] Confirm the latest callback-load fix remains effective.
- [x] Confirm progressive entry already validates a six-metre continuation.
- [x] Recheck the target-loss episode against its last valid longitudinal observation.
- [x] Add a tested stage-corridor/base-bound intersection resolver.
- [x] Export accepted receding-horizon hard bounds.
- [x] Apply the bounds to the OSQP `e_y` state constraints.
- [x] Add configuration and runtime diagnostics.
- [x] Run targeted tests, package tests and build.
- [x] Commit the implementation.

## Verification

- `make autoware-build`: passed (25 packages).
- Stage-corridor focused gtest: 4/4 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 test targets passed.
- `colcon test-result --verbose`: 1227 tests, 0 errors, 0 failures.

## Definition of Done

- The actual tracking QP cannot leave an active validated overtake corridor.
- Reference generation remains soft and continuous.
- Invalid corridor data fails closed.
- Existing ROS/evaluation interfaces remain unchanged.
