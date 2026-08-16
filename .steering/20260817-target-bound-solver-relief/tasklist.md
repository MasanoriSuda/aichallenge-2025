# Tasklist

- [x] Analyze `20260817-065615` and locate target-bound solver churn.
- [x] Separate wall-only and wall-plus-target envelopes.
- [x] Add the target-bound promotion state machine.
- [x] Apply solver-triggered wall-only cooldown.
- [x] Add local/cloud parameters and runtime diagnostics.
- [x] Add focused unit tests.
- [x] Run build and package regression tests.
- [x] Commit without generated run results.

## Definition of Done

- A transient target-bound candidate cannot toggle hard opponent bounds every
  control cycle.
- One target-bound solver failure gives wall-only MPC a recovery window before
  the existing Mission abort threshold.
- Wall bounds never disappear because target-bound promotion is withheld.
- Existing ROS and evaluation contracts remain unchanged.

## Verification

- `make autoware-build`: 25 packages passed.
- Focused gate/corridor tests: 8 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 1207 tests,
  0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
