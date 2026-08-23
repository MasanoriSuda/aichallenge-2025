# Design

## Hypothesis

The five-state QP contains metres, radians, metres per second, acceleration,
curvature and local progress.  OSQP's default termination evaluates an
unscaled global residual.  A large progress/dynamics row can therefore make a
curvature miss of several milliradians per metre globally acceptable even
though the existing per-row execution contract correctly rejects it.

OSQP already equilibrates the problem internally.  Its
`scaled_termination` contract evaluates convergence in that equilibrated
space.  This candidate asks the existing solver to terminate against the
space it actually iterates in; it does not rewrite the QP, change its feasible
set, or repair a rejected primal downstream.

Dynamic evidence rejected this hypothesis as a production correction.  The
equilibrated-space stopping test did not enforce the existing physical-unit
row contract.  In the difficult course segment it instead needed hundreds to
thousands of iterations and still returned physical residuals rejected by the
solver wrapper.

## Scope

Introduce an immutable `SolverConfiguration` at `PersistentOsqpSolver`
construction.  The default remains byte-for-byte equivalent to the current
legacy behavior.  Only `track_cruise_shadow_solver_context_` is constructed
with scaled termination enabled.  Tactical left/right contexts and the shared
legacy solvers retain the default.

The solve result is still returned in physical coordinates by OSQP.  Existing
warm-start, rowwise residual, semantic normalization, physical wall proof and
canonical authority code remain unchanged.

## Competing approaches not selected

- Per-row post-solve acceptance: previously caused a legacy cold-reset cascade.
- Explicit constraint-row normalization: reduced row rejects but increased
  physical rejects, maximum iterations and callback overruns.
- Primal restoration: changed 120--145 fields and was dynamically rejected.
- Tolerance relaxation or clamping: hides the violated execution contract.

## Rejection rule

This is not retained as a compatibility branch.  If dynamic evidence fails,
all source/test changes are removed and only the audit result remains.

The rule fired in `output/20260823-110820`; no solver configuration or test
surface from this candidate remains in production source.
