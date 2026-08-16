# Tasklist

- [x] Analyze the 20260817-062752 trial and identify the ownership gap.
- [x] Generate base wall stage bounds in the common horizon evaluator.
- [x] Preserve tighter target-bound corridors on validated optimizer output.
- [x] Expose wall-only versus wall-plus-target diagnostics.
- [x] Add focused unit coverage.
- [x] Run build and package regression tests.
- [x] Commit without including generated result data.

## Verification

- `make autoware-build`: passed, 25 packages.
- Focused wall-corridor tests: passed, 4/4.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed,
  1230 tests, 0 errors, 0 failures.
- `git diff --check`: passed.

## Definition of Done

- Active Mission execution never drops all QP lateral corridor constraints
  solely because a tactical replan is pending or Return has begun.
- Wall constraints remain hard while opponent constraints can degrade only
  through the existing bounded contact-tolerant policy.
- Existing ROS and evaluation interfaces are unchanged.
