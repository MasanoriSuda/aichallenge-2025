# Canonical numerical-boundary contract requirements

## Purpose

Close the Track/Cruise Gate A mismatch between a numerically certified five-state QP solution and
the strict semantic invariants of `CanonicalExecutionPlan`.  This Slice does not promote runtime
authority and does not tune OSQP or vehicle parameters.

## Dynamic evidence

Run `output/20260822-225811` used the current HEAD `2dd1d33f3cb0ecd08e8725cec80e4c4f839cbbdb`
in one-car `make dev` for five observed waypoint wraps.  It reproduced 324 outcomes with:

- `status=canonical-plan-reject`;
- `plan-contract-rejected/invalid-control-stage`;
- solved, finite and physically certified five-state solutions;
- zero physical wall rejection in the corresponding windows;
- `authority=shadow`, `selected=0` and no control callback overrun in the inspected windows.

The canonical validator checks control-stage acceleration and curvature only for finiteness, stage
duration for positivity, and virtual-progress speed for finiteness plus strict non-negativity.
Because the primal is finite and stage duration is solver-independent, the observed rejection can
only be a negative virtual-progress input admitted within the numerical QP tolerance.

## Requirements

1. Preserve the raw OSQP primal for residual reporting and warm start.
2. Before any executable five-state extraction, validate every semantically non-negative velocity
   state and virtual-progress input against its own box-constraint residual and tolerance row.
3. Normalize a negative value to zero only when both its magnitude and recorded constraint
   violation are within that exact row tolerance.
4. Reject malformed provenance or any tolerance-exceeding value with field, stage, raw value,
   violation and tolerance diagnostics.
5. Make actuation, execution trajectory, physical certificate, legacy comparison conversion and
   canonical plan extraction consume the same normalized execution primal.
6. Keep canonical validation strict.  An executable plan itself must never contain a negative
   semantic speed.
7. Keep production authority on legacy MPC (`authority=shadow`, `selected=0`).
8. Do not change OSQP tolerances, wall margins, costs, horizons or racing parameters.

## Definition of done

- Failure-first unit tests cover within-tolerance normalization, tolerance-exceeding rejection and
  malformed residual provenance.
- Existing tests and package build pass.
- A repeated Track/Cruise shadow run has zero `invalid-control-stage`, reports any numerical
  normalization explicitly, keeps `actuation_diff=0`, and remains physically certified.
- Gate A is updated from evidence.  Retained revalidation remains blocked until the rerun passes.
