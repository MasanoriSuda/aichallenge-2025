# Requirements

## Objective

Make the exact physical continuation and terminal Stop certificate model the
actuation which actually crosses the ROS publisher boundary.

The seven-state SQP owns steering rate internally, but the serialized command
owns steering tire angle. `output/20260830-142647` and
`output/20260830-143906` show that the next canonical command origin equals the
previous published angle while the certificate advances by discrete
steering-rate steps.

## Root cause

Both `build_continuation()` and `build_stop_contingency()` start from the
serialized steering angle and then integrate the same stage steering rate
during the first publisher interval. That rate is not present on the ROS wire.
The certificate therefore proves a different plant input from the command it
authorizes.

## Constraints

- Do not change Mission lifecycle or production authority selection.
- Do not add a flag, lease, grace period, timeout, fallback, tolerance, or
  clearance parameter.
- Preserve acceleration during the publication interval because acceleration
  is serialized.
- Hold the serialized tire angle during the publication interval because
  steering rate is not serialized.
- Resume the future SQP steering-rate sequence only after the publication
  boundary.
- Apply one rule to normal continuation and terminal Stop construction.

## Definition of Done

- Exact continuation holds initial steering over its first publication
  interval.
- Terminal Stop actuation evidence records zero steering rate over that same
  interval.
- Stage-end steering is derived from the exact nonlinear state, not an
  independent rate formula.
- Tests prove the serialized-angle hold and future-rate resumption.
- Build and focused tests pass.
- `make dev2` shows command-origin successor steering error no longer has the
  previous one-rate-step pattern.
