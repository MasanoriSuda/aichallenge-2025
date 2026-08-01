# Task List

- [x] Analyze the P2 Recovery deadlock.
- [x] Define a simulation-only bounded Forward probe policy.
- [x] Implement and integrate the pure policy.
- [x] Add positive and fail-closed unit tests.
- [x] Run Docker build and package tests.
- [x] Record dynamic verification points.

## Static Verification

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: succeeded.
- Package test result: 702 tests, 0 errors, 0 failures, 0 skipped.
- New `StuckRecoverySolverDeadlock` tests: 2 passed.
- `git diff --check`: succeeded.

## Dynamic Verification

Run `make dev2` and inspect a mixed-contact solver Recovery:

- The first failed Reverse-only cycle remains unchanged (`forward_probe=0`).
- After `aggressive_retry: cycle=1`, the same deadlock context reports `forward_probe=1`.
- If a safe Forward candidate exists, the log reports
  `maneuver selected: direction=Forward` with positive contact/course improvement.
- The Forward rollout must report complete V2X data and `corridor_v2x_clear=1` before moving.
- The previous unbounded `maneuver_direction_unknown` retry loop should not recur when an improving
  Forward escape exists.
