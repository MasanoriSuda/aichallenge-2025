# Design

## Time-domain ownership

The retained join has two independent reachability questions:

1. steering: previous published command to the next published command;
2. velocity: observation-time measured speed to control-origin predicted
   speed.

They must not share one duration merely because both checks are performed in
the same function.

The velocity interval is therefore:

```text
observation_to_control_sec = control_origin_sec - now_sec
lower = max(0, current_speed + a_min * observation_to_control_sec - tolerance)
upper = max(0, current_speed + a_max * observation_to_control_sec + tolerance)
```

The steering interval remains:

```text
command_to_command_sec = publication_interval_sec
max_step = steering_rate_limit * command_to_command_sec + tolerance
```

No extra publication period is added to the velocity duration because the
retained actuation's predicted speed is the state at the current control
origin, not a future state after another publication.

## Failure observability

Velocity diagnostics belong to `Result`, not only to accepted `Proof`,
because the values are most important on `VelocityUnreachable`.  The accepted
proof copies the same calculated fields.  Controller shadow telemetry records
the result fields before checking whether a proof exists.

## Non-goals

- production authority promotion;
- dynamic-path availability repair;
- acceleration or tolerance tuning;
- cross-formulation fallback;
- steering reachability redesign.
