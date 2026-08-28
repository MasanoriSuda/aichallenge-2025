# Requirements

## Objective

Remove the unit/ownership mismatch between the nonlinear execution-trajectory
proof and the final current-world wall proof. Both must use one physical
lateral-bound tolerance derived from the actual accepted row residual, not a
global QP scale containing progress, steering and other units.

## Frozen evidence

- Generalized decision 1566 solves internally, but current-world wall proof
  rejects the exact rollout by about 0.001 m.
- Generalized decision 2473 is independently QP-feasible, but its external
  primal also reaches the current-world wall proof with an out-of-bound exact
  rollout.
- `mpcc_rate_resolved_physical_adapter` assigns
  `ExecutionArtifact::physical_global_tolerance` to the exact trajectory's
  lateral-bound tolerance.
- `evaluate_rate_resolved_physical_solution` replaces that value with
  `max(1e-5, maximum_constraint_violation + 1e-6)` before the final wall proof.

## Constraints

- Do not change solver tolerances, iteration limits, wall clearance, obstacle
  clearance, authority, Mission state, fallback, lease, grace or timeout.
- Do not accept a final OSQP iterate that does not satisfy its convergence
  contract.
- Preserve global numerical tolerance for quantities which still explicitly
  own that contract; only lateral geometry is in scope.
- A stricter internal rejection must feed the existing bounded
  post-refinement SQP correction instead of creating a new fallback.

## Definition of Done

- One named resolver owns the physical lateral-bound tolerance.
- artifact validation, fresh nonlinear rollout, retained continuation and
  current-world wall proof consume the same resolver.
- A large mixed-unit global tolerance cannot mask a lateral-bound violation.
- Frozen wall failures are replayed and reclassified without changing
  production authority or configuration.
