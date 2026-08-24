# Requirements

## Objective

Remove the false `velocity-unreachable` retained-admission failures caused by
comparing values from different time origins with the publication-period
reachability budget.

## Root-cause hypothesis

- `Request::current_speed_mps` is measured at `now_sec`.
- retained actuation `predicted_speed_mps` is sampled at
  `control_origin_sec`.
- the current implementation nevertheless uses `publication_interval_sec`
  for the velocity reachability interval.
- the actual observation-to-control interval is already represented by
  `control_origin_sec - now_sec` and by the measured-to-control prefix.

This rejects physically reachable candidates whenever the speed change over
the control prefix is greater than the change possible in one 25 ms command
publication period.

## Constraints

- Do not change acceleration, wall, clearance, solver or cadence parameters.
- Do not add a fallback, retry, feature flag or authority transition.
- Keep steering reachability on the command-to-command publication interval;
  its predecessor value has different provenance from measured velocity.
- Preserve `dynamic-path-blocked` as a fail-closed current-world rejection.
- Keep the rate-resolved path `authority=shadow, selected=0`.
- Do not edit or commit `aichallenge/result-summary.json`.

## Acceptance

- A failure-first test demonstrates that a speed reachable over the exact
  observation-to-control interval is rejected by the old implementation.
- The corrected implementation accepts it and still rejects a truly
  unreachable velocity.
- Rejection telemetry contains the velocity delta, interval bounds and time
  horizon even when no accepted proof exists.
- Full package tests and `make autoware-build` pass.
- A new `make dev2` run materially reduces `velocity-unreachable` without
  weakening dynamic obstacle rejection.
