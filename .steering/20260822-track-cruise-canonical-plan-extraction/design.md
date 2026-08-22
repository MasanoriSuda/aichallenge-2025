# Design

Add a small Eigen-aware adapter library between the solver result and the Eigen-free execution-plan
contract.

```text
Certified extended primal
  states: [e_y, e_lag, e_psi, v, local theta] x (N + 1)
  inputs: [a, kappa, v_theta] x N
  stage duration x N
        |
        v
CanonicalExecutionPlan
  states: [e_y, e_lag, e_psi, v, absolute progress]
  inputs: [a, kappa, v_theta, duration]
```

The adapter does not call `convert_extended_solution_to_legacy()`. It uses the canonical dimension
and index constants, restores `absolute progress = local theta + progress_origin_m` once, then calls
`validate_canonical_execution_plan()` as the final acceptance gate.

The result carries an explicit extraction reason and, for downstream traceability, the nested plan
contract reject reason when final validation fails.
