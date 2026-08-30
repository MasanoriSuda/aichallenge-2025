# Design

## Cause and propagation

1. The seven-state Track/Cruise SQP solves from the measured control-origin
   state.
2. Its exact continuation and swept-footprint wall proof are accepted.
3. Terminal Stop synthesis samples a separate progress-aligned approximate
   support.
4. That conservative support excludes an exact-grid-clear state at sample 0,
   or later in the braking rollout.
5. Stop synthesis aborts before occupancy-grid wall proof.
6. The atomic authority join retains the external Stop indefinitely.

## Change

Keep `StopCourseGeometry` immutable and use it for path interpolation and
diagnostics. During nonlinear Stop construction, record the first and maximum
departure from that approximate support. Do not reject the Stop on that
departure alone. The exact trajectory is then passed unchanged to the
existing swept-footprint occupancy-grid proof and timed dynamic-obstacle
proof, which remain the only physical acceptance authorities.

This removes duplicate wall ownership rather than adding a special start
case. The same rule applies to current-world retained revalidation, Stop
successor construction and offline comparison because all production paths
already require the exact downstream certificate.

## Non-goals

- No clearance relaxation.
- No expansion of the physical occupancy grid or footprint.
- No retention/grace period.
- No special D2/start-position branch.
- No change to normal SQP or Emergency Stop publishing.
