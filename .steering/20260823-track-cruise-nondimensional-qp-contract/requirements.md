# Track/Cruise nondimensional QP contract

## Objective

Remove the mixed-unit convergence defect at its formulation boundary.  The
five-state Track/Cruise QP must enter OSQP in one coherent dimensionless
coordinate system and must leave the adapter in the existing physical-unit
contract.

This is not another row-only normalization experiment.  Variables,
constraints, objective, primal warm start and dual warm start must be
transformed by one immutable algebraic mapping.

## Root-cause boundary

Valid six-lap run `output/20260823-104739` showed 109 strict execution-primal
rejects after OSQP reported `solved`; 80 were stage-zero curvature.  Existing
audits proved that a mixed-unit global residual allows progress-scale rows to
relax small actuator rows.

Rejected alternatives must not be repeated:

- absolute-only OSQP termination;
- row-only normalization;
- row normalization plus ad-hoc dual rebase;
- OSQP polish;
- downstream primal restoration;
- scaled termination.

## Constraints

- Preserve the physical QP feasible set and physical objective exactly.
- Derive one scale per semantic state/input dimension and repeat it across the
  horizon; do not create stage-dependent units.
- Derive one row scale from the semantic row dimension: state dynamics, box,
  and curvature-rate.
- Transform `P`, `q`, `A`, `l`, `u`, primal and dual consistently.
- Return primal, dual, residual and tolerance in physical units.
- Apply the candidate only to the dedicated Track/Cruise production context.
- Leave tactical left/right and legacy/shared solvers unchanged.
- Add no YAML parameter, runtime flag, fallback, retry, timeout or tolerance.
- Keep all semantic normalization and physical certificates unchanged.

## Acceptance

- Failure-first tests prove that the old API cannot express this contract.
- Round-trip tests prove physical primal/dual and QP equivalence.
- Malformed/non-positive scale vectors fail before solver state is mutated.
- Complete build and all package tests pass.
- A short dynamic gate has no continuous solve-failure cascade, callback
  overrun, physical reject, contact, stuck confirmation or Recovery.
- If the short gate passes, six laps materially reduce the baseline 109
  execution-primal rejects without creating a new tail failure.

If any dynamic gate fails, remove all production/test code and retain only the
audit evidence.
