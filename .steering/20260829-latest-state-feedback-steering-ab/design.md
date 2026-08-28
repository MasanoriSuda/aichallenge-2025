# Design

## Hypothesis

The asynchronous preparation remains useful, but its current desired steering
may lie outside the command that can physically cross the next publication
boundary.  Returning immediately discards the entire prepared suffix.  A fast
feedback phase should instead solve

```text
minimize 0.5 * (delta_feedback - delta_prepared)^2
subject to
  -delta_max <= delta_feedback <= delta_max
  delta_previous - rate_max * age - certificate_tolerance
    <= delta_feedback <=
  delta_previous + rate_max * age + certificate_tolerance
```

This one-dimensional convex problem has an exact interval-projection solution.
The observation arm then starts the existing nonlinear continuation from that
feedback steering while retaining the prepared acceleration, steering-rate,
progress-rate and all future stages.

## Why this is not a clamp patch

The projected value is not published and does not weaken a constraint.  It is
the analytical solution of the feedback subproblem under the unchanged
physical envelope.  Promotion requires rebuilding a complete certified
feedback artifact and passing current wall/dynamic proofs; that later Slice
must delete the old elapsed-suffix-only rejection/adoption branch atomically.

## Non-goals

- No full synchronous solve in the 40 Hz callback.
- No production command correction.
- No wall/dynamic proof bypass.
- No change to candidate generation or tactical selection.
