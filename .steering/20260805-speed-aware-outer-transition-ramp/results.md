# Results

## Implemented

- Admission-time handoff sizing now uses the larger of current ego speed and
  the selected overtake command speed.
- The stored admission shift distance is diagnostic/nominal. Runtime no longer
  uses it as a hard cap.
- Runtime derives the maximum ramp from the remaining scheduled window, the
  configured 8 m maximum and the remaining absolute Pass budget.
- The actual ramp is recalculated from current lateral error, current speed and
  the existing 6.0 m/s^2 lateral-acceleration limit.
- Existing target-front, footprint prediction, V2X freshness, wall, rollout,
  rear-clear and Return checks still run before the side change is committed.
- Rejections now expose speed, lateral adjustment, required shift distance and
  available shift distance.

No configuration, ROS interface, global acceleration or braking value changed.

## Static verification

- `make autoware-build`: passed; 25 packages built.
- Focused `test_v2x_overtake_core`: passed.
- Full `multi_purpose_mpc_ros` package test: 25/25 targets passed.
- Test-result summary: 881 tests, zero errors/failures/skips.
- `git diff --check`: passed.

The build emitted only the existing setuptools deprecation warning. The final
test-result scan also reported the existing stale
`build/joycon_contract_guard/package.xml` lookup warning; test execution and the
881-test summary were successful.

## Dynamic acceptance criteria

In the next `make dev2` run, inspect `d1/autoware.log` for:

1. `PassPlan frozen` with `outer_transition=1` and a finite nominal
   `transition_shift`;
2. `scheduled outer transition accepted` with finite `shift`,
   `nominal_shift`, and `available_shift`;
3. the actual `shift` remaining within `available_shift`, even when it is
   longer than the admission-time nominal value;
4. no repeated `same-side lateral ramp exceeds acceleration budget` while
   several metres of transition window remain;
5. side handoff followed by rear-clear and `Pass -> Return -> Idle`;
6. no contact, wall Recovery, SafetyBrake pause or sustained hard braking from
   that mission.

