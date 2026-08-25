# Requirements

## Purpose

Promote `ControlIntent::Rejoin` from its dedicated five-state normal owner to
the shared steering-rate-resolved six-state MPCC owner, then physically delete
the replaced Rejoin lifecycle in the same Slice.

## Root evidence

- `OvertakeLinePhase::Recovery` is resolved by the orchestrator to
  `ControlIntent::Rejoin`.
- Track, Cruise, Follow, ShiftOut, Pass and Return already publish through
  `velocity-steering-progress-6state`.
- Rejoin alone still allocates a private five-state solver context, warm-start
  identity, plan store, evaluator and telemetry window.
- The six-state semantic builder already receives the Recovery-line racing
  reference, stage bounds and velocity horizon; it is omitted only by the
  shared intent/scope capability boundary.

## Required behavior

- Rejoin uses the same six-state formulation, solver/artifact store and
  causal publication boundary as every other normal intent.
- Rejoin semantics remain the base racing-line Recovery reference; no target
  or execution-side identity is invented.
- Retained Rejoin execution must pass the existing current wall and complete
  dynamic-obstacle-world proof. Artifact age alone is never sufficient.
- Missing semantic, fresh, retained or physical evidence selects explicit
  Emergency; it must not restore the five-state owner.
- The dedicated five-state Rejoin solver, warm state, plan store, evaluator,
  production telemetry and dispatch branch are physically deleted.

## Non-goals

- No wall, solver, horizon, weight, speed or timeout tuning.
- No change to Stuck/AWSIM Reverse or `LowSpeedRejoin` recovery authority.
- No new fallback, flag, lease or retained-age exception.
- No change to ROS topics, services or result schemas.

## Gate

- Failure-first tests prove Rejoin scope was absent at baseline.
- All normal intents resolve through one six-state capability and dispatch.
- Source-contract tests prove the old five-state Rejoin owner cannot be
  reconnected.
- Package tests and `make autoware-build` pass.
- A bounded `make dev2` run exercises Rejoin and shows only six-state normal
  authority or explicit Emergency, with no five-state Rejoin publication.
