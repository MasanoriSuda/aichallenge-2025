# Task list

- [x] Record the post-fix run evidence and scope.
- [x] Add normal-latch and live-TTC semantics to the pure handoff resolver.
- [x] Add the prediction-uncertainty speed-reference policy.
- [x] Wire controller logging and runtime speed ownership.
- [x] Add focused unit tests.
- [x] Run focused tests and `make autoware-build`.
- [x] Record verification and remaining dynamic checks.

## Verification

- `make autoware-build`: passed; 25 packages finished.
- `V2XOvertakeCoreMissionOwnership.*`: 31/31 passed.
- `test_v2x_overtake_core`: 497/497 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 1001 tests,
  0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

## Dynamic check remaining

Run `make dev2` and require:

- handoff releases predominantly with `normal_latch` rather than `expiry`;
- handoff `current_overlap` releases decrease from 3;
- `prediction_hold=1` preserves Overtake without accelerating into overlap;
- `Pass -> Return -> Idle` remains available;
- SafetyBrake, Recovery, Reverse, and OSQP failure counts do not increase.
