# Design

## Boundary

The live callback seals the solver request together with current pose,
course-frame knots, wall-grid ownership and footprint into one immutable
pipeline snapshot, then submits it to the existing one-running/one-pending
latest-only solver worker.

That one worker serially performs solve, exact trajectory adaptation and the
complete endpoint/swept-footprint proof. It publishes immutable solver and wall
results carrying the same artifact sequence, decision, intent and stage
geometry; the wall result additionally carries the sealed world snapshot
identity. The live callback consumes results without waiting.

## Ownership

```text
rate-resolved solver worker
  -> immutable six-state artifact
  -> exact physical trajectory adapter
  -> complete wall certificate against sealed current-world snapshot
control callback
  -> provenance check + telemetry only
```

The wall grid is immutable after map construction and must have shared lifetime
ownership. A raw pointer may not escape into the worker.

A separate physical-proof worker is intentionally forbidden. Dynamic run
`output/20260825-044053` showed correct proof results but increased D1
observation-only callback overruns from 2 to 21 through concurrent scheduling
contention. Serial work in the existing worker preserves non-blocking control
without adding another 40 Hz execution thread.

## Non-goals

- retained actuation;
- production authority;
- proof cadence tuning;
- wall-margin tuning;
- legacy five-state wall-path deletion.
