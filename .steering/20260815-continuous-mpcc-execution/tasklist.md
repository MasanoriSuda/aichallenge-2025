# Task list

- [x] Inspect the latest run's ShiftOut/Pass/FollowPrepare failure transitions.
- [x] Refactor Pass-only Frenet-DP authority into continuous execution authority.
- [x] Preserve a validated DP path across eligible DynamicMissionWait entry.
- [x] Execute and revalidate the retained DP path during rolling replan.
- [x] Bridge exact-target DP authority into front-danger arbitration.
- [x] Keep local and cloud configuration comments aligned.
- [x] Run core unit tests (1131 tests, 0 failures).
- [x] Build `multi_purpose_mpc_ros` in the development container.
- [x] Review the final diff; no ROS/evaluation interface changes found.
- [x] Commit the change.

## Dynamic confirmation requested

- Compare `planning_unavailable`, `best=none`, and DynamicMissionWait counts.
- Confirm `continuous_dp=1` is present during a soft planning miss.
- Confirm center-distance SafetyBrake does not interrupt that prefix for the
  exact locked target.
- Confirm physical wall/contact and different-front-vehicle hazards still
  revoke authority.
- Compare `Pass -> Return` completion count and Recovery count with
  `20260815-121327`.
