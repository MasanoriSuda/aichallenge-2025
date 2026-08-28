# Requirements

## Objective

Determine whether the frozen ShiftOut wall-refinement failure can be resolved
by rebuilding the SQP tangent from a wall-directed feasibility seed, without
changing production authority or any physical/numerical tuning parameter.

## Constraints

- Production `SolverContext::evaluate()` remains byte-for-byte equivalent in
  behavior unless the audit-only entry point is explicitly called.
- Do not change wall clearance, trust bucket widths, OSQP settings, Mission
  lifecycle, lease, grace, timeout, or fallback behavior.
- A relaxed restoration QP is never an execution artifact. It may only seed a
  fresh full wall refinement.
- A result is accepted only after the unchanged full seven-state QP, exact
  nonlinear trajectory adapter, wall proof, dynamic proof, and terminal
  successor proof pass.
- Use the immutable interaction fingerprint and do not publish commands.

## Definition of Done

- The architecture comparator has a separately labelled wall-restoration arm.
- Tests prove that the normal production evaluation path cannot enable the
  restoration arm accidentally.
- Frozen decision 2473 is classified as restoration-success, remaining
  single-SQP limitation, certificate mismatch, or physical infeasibility.
- The finding and next architecture decision are recorded before any
  production integration.
