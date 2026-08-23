# Requirements

## Evidence boundary

- Baseline: `f0fe5c8`
- Runtime evidence: `output/20260824-020904`
- Domains: D1 and D2
- Preserve user-owned `aichallenge/result-summary.json`.

## Observed phenomenon

The Track/Cruise five-state solver reports `OSQP_SOLVED`, but downstream
execution-primal certification rejects acceleration, predicted velocity,
curvature, or virtual-progress-speed box rows. The accepted solver result does
not currently retain OSQP's successful primal/dual residuals or the global
physical tolerance that allowed the result through the common adapter.

## Scope

- Add provenance-only telemetry at the OSQP success boundary.
- Carry that telemetry to the Track/Cruise final outcome log.
- Record the OSQP settings relevant to termination and the physical row report
  derived from the exact returned primal.
- Add deterministic tests for the telemetry contract.

## Non-scope

- No solver setting, tolerance, scaling, cost, bound, warm-start, authority,
  fallback, retained-plan, or command behavior change.
- No normalization or clamping of a rejected execution primal.
- No Track/Cruise row-preconditioning promotion.

## Exit gate

- A successful solve exposes finite OSQP primal/dual residuals, termination
  settings, row-preconditioning provenance, physical global tolerance, and the
  worst per-row normalized violation.
- Track/Cruise rejection logs join those fields to the exact decision and
  rejected semantic field.
- Focused test, package test, and package build pass.
- A bounded `dev2` run supplies enough evidence to distinguish solver
  convergence/scaling mismatch from a boundary-saturated QP formulation.
