# Tasklist

- [x] Inspect latest run and identify dominant Recovery paths.
- [x] Confirm current wall-clock and distance budget ownership.
- [x] Add pure prediction-grace resolver and unit tests.
- [x] Add active-Pass time accounting across pause/replacement.
- [x] Add configuration and concise runtime logging.
- [x] Run focused unit tests.
- [x] Build `multi_purpose_mpc_ros` in the development container.
- [x] Record dynamic verification criteria.

## Definition of Done

- Prediction-only loss receives at most the configured bounded grace.
- Any hard guard failure remains immediately unsafe.
- Paused/replacement ShiftOut time does not count as active Pass time.
- Active Pass time is preserved across replacement and cannot be reset.
- Focused tests and package build pass.

## Verification

- `make autoware-build`: passed (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose`: 915 tests, 0 errors, 0 failures.

## Next dynamic run

Compare against `output/20260808-070409` and confirm:

- `prediction-only guard grace started` is followed by either rear-clear/Return or a bounded
  hard/expired abort; it must never continue through current-body overlap, wall contact, Emergency,
  or solver Recovery.
- `SafeSeparation aborted: short horizon unsafe` decreases from six occurrences.
- An alternate `PassPlan replaced` does not immediately fail because paused/ShiftOut wall time
  consumed the 10-second Pass ceiling.
- `Pass -> Return -> Idle` increases from two occurrences without increasing contact or wall
  Recovery.
