# Design

## Causal repair

```text
physical five-state QP
  x = [e_y, e_lag, e_psi, v, s, a, kappa, v_theta]
  -> derive positive characteristic scale D from its physical variable bounds
  -> solve z in x = D z
  -> Pz = D P D, qz = D q, Az = S A D, lz = S l, uz = S u
  -> warm primal z0 = D^-1 x0, warm dual y0_solver = S^-1 y0_physical
  -> recover x = D z, y_physical = S y_solver
  -> certify original A, l, u row by row
```

`D` is a coordinate transform, not a change to the feasible set or controller
weights. Characteristic scales come from finite state/input box bounds already
owned by the five-state problem schema. An unbounded or degenerate coordinate
uses the neutral scale 1.0. No runtime tuning parameter is introduced.

`S` maps the strictest finite side of each physical row tolerance to OSQP's
absolute tolerance. The physical tolerance remains `eps_abs + eps_rel *
abs(boundary)`, but the normalized solver uses `eps_rel=0`: applying a global
relative term again would let a distant opposite bound or progress row relax a
small-unit row. This was observed in `output/20260824-025957` and
`output/20260824-030745`; acceleration `[-3, 1.37]` and virtual progress
`[0, about 11]` were solved against their distant side while the physical
certificate correctly checked the crossed side.

Variable scaling is additionally required because row-only scaling was
dynamically rejected in `output/20260824-011002`.

## Ownership boundary

The common solver receives an explicit immutable variable-scale vector from
the five-state problem producer. It does not guess units from arbitrary sparse
rows. Legacy three-state calls omit it and keep their existing behavior until
their authority is deleted.

All five-state contexts use the same contract:

- Track/Cruise;
- Follow;
- live Overtake/DynamicEscape;
- left/right tactical branch solvers.

## Warm-start and dual contract

Stored certified warm starts remain in physical coordinates. Scaling exists
only inside `PersistentOsqpSolver`, so stage transport and progress rebasing do
not acquire a second unit convention. On every solve:

- physical primal warm start is divided by `D`;
- physical dual warm start is divided by `S`;
- returned primal is multiplied by `D`;
- returned dual is multiplied by `S`.

## Alternatives rejected

- `scaled_termination=true`: does not prove the physical row contract.
- lower physical tolerance or more iterations: parameter tuning, not a
  coordinate fix.
- post-solve clamp: hides dynamics/rate inconsistency.
- cold retry: adds a second execution path.
- infer units inside the generic solver from column magnitudes alone: couples
  safety semantics to incidental sparsity and cost weights.

## Failure-first test

Use an N-stage coupled mixed-unit problem plus asymmetric `[-3, 1.37]` and
`[0, 12]` bounds. The pre-fix contract scales from the distant side (250 rather
than the strict-side mapping 1.0) and fails the test. The repaired policy
embeds the physical relative tolerance exactly once, solves once, and
certifies every physical row. A separate algebraic test verifies primal/dual
round trips.
