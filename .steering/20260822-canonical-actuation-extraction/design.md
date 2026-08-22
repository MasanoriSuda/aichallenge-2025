# Design

Add an Eigen-free command-extraction boundary to the canonical plan contract:

```text
CanonicalExecutionPlan + exact CanonicalExecutionCursor + wheelbase
  -> state[i + 1].velocity
  -> input[i].acceleration
  -> input[i].curvature
  -> atan(wheelbase * curvature)
  -> input[i].virtual_progress_speed
```

The result retains plan ID and stage index for final-command provenance. Exhausted or mismatched
cursors fail closed.

Track/Cruise shadow evaluates this after fresh admission and compares it with the direct primal
proposal. The extracted result remains telemetry-only in this Slice.
