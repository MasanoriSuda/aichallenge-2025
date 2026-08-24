# Requirements

## Objective

Build the complete numerical skeleton for a six-state, steering-rate-input
MPCC as an isolated shadow QP. The component must own dynamics, variable boxes,
quadratic references, input-delta costs, row semantics, and physical-unit
coordinate scaling without being linked into the production controller.

## Invariants

- State layout is exactly `[e_y, e_lag, e_psi, v, theta, delta]`.
- Input layout is exactly `[a, delta_dot, v_theta]`.
- State zero is a hard equality to the observation snapshot.
- Steering angle has one owner: its state box.
- Steering rate has one owner: its input box.
- There is no curvature input or duplicate curvature-rate row.
- Every state/input reference, bound, and weight is stage-major and exact-size.
- Scaling is derived from the same physical box bounds returned by the QP.
- Malformed or non-convex input fails closed; values are never repaired.

## Constraints

- Do not link the new QP assembler into `mpc_controller_cpp`.
- Do not add runtime config, authority, fallback, timeout, or tuning.
- Do not change the established five-state formulation or solver path.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- Pure QP assembly and row decoding have deterministic unit tests.
- A small straight-line problem solves through the existing persistent OSQP
  wrapper and its certified primal respects steering/rate bounds.
- The six-state warm-start layout is accepted by the existing generic shift
  operation without changing production stores.
- Build and full package tests pass.
- Production link audit remains negative.
