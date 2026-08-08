# Requirements

## Purpose

Latest `make dev2` evidence shows that dynamic Mission reselection is now active, but Pass
completion still fails mainly through `SafeSeparation aborted: short horizon unsafe`.
An alternate Mission can also inherit wall-clock time spent outside Pass and immediately hit the
absolute Pass time limit.

## Required behavior

- Preserve immediate failure for physical/runtime hard faults:
  - current confirmed vehicle overlap
  - static-wall contact or unavailable wall sample
  - target position jump or pass-side intrusion
  - EmergencyBrake
  - solver Recovery
- Treat prediction-only loss as a bounded soft fault after forward completion has already latched.
- Soft prediction grace is permitted only while:
  - current vehicle footprints remain strictly separated
  - the execution corridor remains available
  - forward progress is recent
  - the configured grace duration has not expired
- Preserve the absolute Pass time and distance ceilings.
- Count only time actually spent in the `Pass` phase toward the absolute Pass time ceiling;
  `FollowPrepare`, replacement `ShiftOut`, and other paused phases must not consume it.
- Add focused unit coverage and observability without changing ROS topic/service contracts.

## Constraints

- Participant implementation only.
- Do not change `a_max: 1.0`.
- Do not weaken wall, current-body overlap, EmergencyBrake, or solver guards.
- Preserve the user's uncommitted `v2x_overtake_line_min_wall_clearance` A/B value.

