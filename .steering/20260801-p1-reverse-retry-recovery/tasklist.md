# Task List

- [x] Analyze the P1/P2 Reverse retry loop.
- [x] Define bounded stale-latch and measured-course-progress policies.
- [x] Implement solver Reverse-only stale-latch release.
- [x] Implement measured Reverse course-progress stop and Forward reassessment.
- [x] Add positive and fail-closed unit tests.
- [x] Run package tests, build, and `git diff --check`.
- [x] Record dynamic verification points.

## Static Verification

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets succeeded.
- Test result: 757 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: succeeded.

## Dynamic Verification

Run `make dev2` and reproduce a P1/P2 stopped-front contact:

- initial coordinated Recovery still selects Reverse;
- if Reverse improves clearance and course position, normal Recovery is unchanged;
- if measured lateral error worsens by more than 0.10 m outside the rejoin envelope, the log emits
  `measured course progress worsened`, stops Reverse, and reassesses in Drive;
- the next maneuver is Forward only when static/V2X/boost gates report clear;
- `forward_fallback=1` appears after a fully checked blocked Reverse attempt even if the solver has
  already recovered;
- the previous repeated `maneuver_direction_unknown` aggressive-retry loop does not recur.
