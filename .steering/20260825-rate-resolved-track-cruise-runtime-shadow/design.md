# Design

## Root cause and evidence boundary

The rejected `20260824-canonical-stage-publishability` experiment established
that restricting each five-state coarse prediction stage to one 40 Hz steering
step makes ordinary Track/Cruise infeasible. The earliest violated invariant is
the actuator time-base contract, not a wall margin or OSQP tolerance.

The replacement is measured without authority:

```text
Track/Cruise semantic construction
  -> five-state canonical solve and production (unchanged)
  -> immutable six-state shadow request
  -> LatestOnlyWorker
  -> adapter -> QP assembly -> dedicated OSQP context
  -> typed shadow mailbox and telemetry only
```

## Semantic ownership

The existing `build_extended_progress_problem()` remains the single producer
of lateral/lag/heading/velocity/progress references, bounds and costs. During
that same construction it materializes the semantic adapter request. This
avoids reconstructing objectives from sparse `P/q` matrices or duplicating
policy in the worker.

The legacy stage-zero curvature box is narrowed by a 40 Hz publisher workaround.
That intersection is deliberately not copied into the six-state request. The
original physical curvature box is converted to a steering-state box; current
steering is state zero and bounded steering rate is the sole reachability
constraint.

## Async identity

Each immutable snapshot carries:

- monotonically increasing shadow sequence;
- source five-state problem fingerprint;
- decision ID, intent and stage-geometry ID;
- submission time and publication interval;
- the complete semantic adapter request.

The mailbox rejects invalid timing, unregistered sequence, rollback and source
identity loss. Results are diagnostic artifacts and cannot enter any canonical
plan store or final authority selector.

## Solver policy

A dedicated persistent OSQP instance uses the established row-tolerance
preconditioner. Initial runtime measurements are intentionally cold-started on
every numerical update: silently reusing an unshifted prior horizon would be an
invalid warm start. Exact six-state stage shifting plus progress-origin rebasing
is a later approved Slice.

## Acceptance

- Deterministic tests cover mailbox monotonicity, rejection provenance and an
  actual six-state OSQP solve with bounded 40 Hz actuation sampling.
- The package build and full package test suite pass.
- Static source audit shows the result is consumed only by telemetry.
- `make dev2` shows Track/Cruise submissions and completed results without
  control callback ownership or authority changes.
