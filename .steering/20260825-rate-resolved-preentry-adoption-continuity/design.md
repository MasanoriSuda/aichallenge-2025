# Design

## Earliest uncertain boundary

The selected six-state pre-entry plan is solved from an asynchronous snapshot.
At live adoption, the retained revalidator advances the artifact cursor to the
current control origin and joins it to the current vehicle state. The current
log records only the terminal reason, so these distinct cases are conflated:

1. the vehicle legitimately diverged from an old candidate;
2. circular progress was lifted against the wrong origin;
3. steering reachability used inconsistent actuation times;
4. velocity reachability used an inconsistent delay prefix.

## Observation-only evidence

`mpcc_rate_resolved_retained_revalidation::Result` will expose intermediate
values already computed by `evaluate()`, including on rejection. The live
pre-entry shadow copies them into its telemetry record and prints them beside
the artifact time provenance.

```text
snapshot/prediction/control/cursor
measured/lifted/expected progress and tolerance
current/expected steering, delta and reachable step
current/expected speed, delta, reachable interval and duration
```

No second acceptance calculation is introduced. `proof` remains the only
acceptance evidence, and no field added by this Slice is consumed by control.

## Promotion rule

This Slice cannot promote pre-entry authority. A later repair is allowed only
when a dynamic trace identifies the earliest violated invariant and a
deterministic failure-first test reproduces it.
