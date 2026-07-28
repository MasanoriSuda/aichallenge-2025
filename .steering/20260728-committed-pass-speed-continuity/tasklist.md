# Task list

- [x] Inspect latest P1 log and identify cap reapplication / low-reference path.
- [x] Define hold-only front-cap behavior and safety precedence.
- [x] Add pure policy inputs and unit tests.
- [x] Wire configuration and reference-only speed floor into the MPC.
- [x] Run package tests and build.
- [x] Review the final diff and document effect-confirmation points.

## Validation

- `make autoware-build`: success (25 packages).
- Post-build package test: 23/24 CTest entries passed; the only failure was an incorrect
  state setup in the newly added hysteresis test.
- Rebuilt `test_v2x_overtake_core` after correcting that setup: 198/198 passed.
- Final `colcon test-result` after refreshing the corrected result: 634 tests, 0 failures.

## Runtime confirmation

In the next `make dev2` run, verify P1 logs show:

- `phase=Pass`, `cap_release=1`, and `horizon_release=1` when a lateral clamp is safely accepted;
- `floor_active=1` and `v_floor=3.00` after the locked target drops below `1.0 m/s`;
- no new wall-contact, EmergencyBrake, solver-Recovery, or collision events;
- shorter `Pass` duration than the approximately 23 seconds observed in
  `output/20260728-003305`.
