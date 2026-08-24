# Requirements

## Objective

Define one pure semantic bridge from the established five-state Track/Cruise
problem snapshot to the six-state steering-rate-input shadow QP. The bridge
must preserve physical references, bounds and cost meaning without reading ROS,
controller state or configuration globals.

## Invariants

- Five-state state fields map unchanged to the first five six-state fields.
- Current measured steering is the hard sixth state at stage zero.
- Curvature references and boxes map through `delta=atan(wheelbase*kappa)`.
- Legacy curvature tracking cost becomes steering-state tracking cost using the
  local `d(kappa)/d(delta)` Jacobian.
- Legacy curvature stage-difference cost becomes steering-rate magnitude cost
  using the same Jacobian and immutable stage duration.
- Acceleration and virtual-progress costs/bounds retain their existing units.
- The adapter adds no clamp, fallback, runtime flag, timeout or tuned value.
- Malformed semantic input fails closed.

## Scope

- Track/Cruise semantic conversion only.
- Unit and deterministic solver tests only.
- No production controller linkage, ROS logging, authority or dynamic trial.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- Exact mapping and physical-unit cost conversion are tested.
- A curved deterministic snapshot solves with steering and steering-rate boxes
  satisfied.
- Invalid sizes, steering, wheelbase, stage duration and curvature boxes fail.
- Full package tests and production non-link audit pass.
