# Design

## Boundary

The live callback converts a current-semantic six-state artifact into an exact
trajectory, seals current pose, course-frame knots, wall-grid ownership,
footprint and tolerance into an immutable request, then submits it to a
one-running/one-pending latest-only worker.

The worker performs the complete endpoint and swept-footprint proof. It
publishes one immutable result carrying the source artifact sequence, decision,
intent, stage geometry and world snapshot identity. The live callback may
observe a result only when those identities remain compatible; it never waits
for a worker.

## Ownership

```text
rate-resolved solver worker
  -> immutable six-state artifact
control callback
  -> exact adapter + current-world snapshot + submit only
physical proof worker
  -> complete wall certificate
control callback
  -> provenance check + telemetry only
```

The wall grid is immutable after map construction and must have shared lifetime
ownership. A raw pointer may not escape into the worker.

## Non-goals

- retained actuation;
- production authority;
- proof cadence tuning;
- wall-margin tuning;
- legacy five-state wall-path deletion.
