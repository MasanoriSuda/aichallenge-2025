# Task list

- [x] Confirm the latest P1 failure sequence and parameter ownership.
- [x] Define full-clearance initial release versus hysteresis hold.
- [x] Implement the pure policy and controller wiring.
- [x] Add regression tests.
- [x] Build and run package tests.
- [x] Record runtime confirmation points.

## Validation

- `make autoware-build`: success, 25 packages.
- `multi_purpose_mpc_ros`: 24/24 CTest entries passed.
- `colcon test-result`: 634 tests, 0 failures.
- `git diff --check`: clean.

## Runtime confirmation

In the next `make dev2` run, verify:

- after `Pass front-overlap exclusion latched` at lateral separation `>= 1.50 m`,
  `OvertakeLine execution front cap: Released` appears even when `horizon_clear=0`;
- the release reason is
  `physical lateral clearance; constrained feasible Pass horizon accepted`;
- debug shows `cap_release=1, horizon_release=1`;
- once a target falls below `1.0 m/s`, `floor_active=1, v_floor=3.00`;
- `Pass -> Return` occurs and no additional wall-contact, EmergencyBrake, collision, or
  physically-infeasible Recovery is introduced.
