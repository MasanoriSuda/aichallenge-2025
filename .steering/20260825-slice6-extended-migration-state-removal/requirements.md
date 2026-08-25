# Requirements

## Purpose

Remove the inert circuit-breaker, reentry, cross-formulation handoff and
telemetry state left behind when the synchronous extended/three-state/legacy
normal solve chain was deleted.

## Earliest violated invariant

Migration state may influence canonical admission only when a reachable
producer updates that state from the same production solve contract. The old
extended solver producer is gone, but `ExtendedSolverCircuitBreaker`,
`ExtendedSolverReentryGate` and `ExtendedModeHandoff` remain compiled. Their
only controller operations are reads of the never-activated circuit and
resets. The resulting branches and YAML keys describe an authority transition
which can no longer occur.

## Scope

- Delete the unused extended circuit breaker, reentry gate and mode-handoff
  types, implementations, controller state and dedicated tests.
- Delete the migration-only extended runtime telemetry and dead RTI telemetry
  recorder whose producers no longer call them.
- Delete configuration keys used only by those retired responsibilities.
- Replace circuit-degraded inputs to current DP authority contracts with the
  actual invariant: no alternate live extended formulation exists.
- Keep the canonical five/six-state solvers, wall refinement, asynchronous
  branch contexts, Recovery and final publisher unchanged.

## Non-scope

- No parameter tuning, solver setting change or wall-margin change.
- Do not delete the retained legacy DynamicEscape/wall-handoff graph in this
  Slice; it crosses the final publisher and Recovery boundary and requires a
  separate audit.
- Do not change ROS topics, services or message types.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- A failure-first source test rejects restoration of the retired migration
  state, recorder functions and YAML keys.
- No circuit/reentry/handoff object can affect DP authority or normal output.
- Focused tests, full package tests and `make autoware-build` pass.
- The Slice adds zero normal authority, fallback, lease, timeout or feature
  flag.
