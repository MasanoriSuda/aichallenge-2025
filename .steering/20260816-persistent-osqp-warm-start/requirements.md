# Requirements

## Purpose

Keep the main tracking QP temporally continuous and remove per-cycle OSQP
workspace construction without changing race tactics, MPC weights, speed
limits, overtake admission, Return, or Recovery policy.

## Scope

- Reuse an OSQP workspace while matrix dimensions and sparsity are unchanged.
- Update numeric matrix/vector data through the OSQP update API.
- Warm-start the next QP from a one-stage shifted successful primal/dual
  solution.
- Rebuild from a cold workspace after a structural change, failed update, or
  failed solve.
- Emit bounded aggregate timing/iteration diagnostics and control-callback
  overrun diagnostics.
- Preserve `/control/command/control_cmd`, launch, Domain, evaluation and
  submission contracts.

## Out of scope

- MPCC progress state or progress trust region.
- SQP mixing or nonlinear vehicle-model replacement.
- Tactical worker asynchronous execution.
- Frenet-DP, longitudinal policy, Return or Recovery behavior changes.

## Constraints

- Existing changes in `aichallenge/result-summary.json` are user-owned and
  must not be modified or staged.
- A stale or malformed warm start must be ignored; it must never make a valid
  QP fail.
- A persistent-workspace update failure must fall back to one cold setup in
  the same control cycle.
