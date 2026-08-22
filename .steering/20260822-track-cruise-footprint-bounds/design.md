# Design

## Root causal chain

```text
mixed-unit QP
  -> global residual scale dominated by course progress
  -> metre-domain lateral error can pass wrapper acceptance
  -> extractor uses observed violation as tolerance
  -> unsafe state reaches footprint certificate
```

The physical certificate is the acceptance oracle and is not weakened.

## Accepted contract

`PersistentOsqpSolver` computes a row-wise report after every returned solution:

```text
violation_i = distance(A_i x, [l_i, u_i])
tolerance_i = multiplier * (eps_abs + eps_rel * max(|A_i x|, |projection_i|))
```

The wrapper preserves its existing global acceptance behavior during shadow migration, but records
the row vectors and worst normalized row. Track/Cruise five-state shadow then applies a semantic
metre-domain contract to the stage 1..N lateral box rows. Pose extraction uses the largest permitted
lateral-row tolerance, not the measured residual.

This keeps solver convergence policy separate from safety-unit acceptance and makes future
authority promotion auditable.

## Why physical preflight was not retained

Three variants were measured:

1. Post-solve exact-heading contraction and second solve. It doubled same-cycle work; most second
   solves either exceeded their lateral row or reached maximum iterations.
2. Reference-heading `e_y` box contraction before one solve. It treated a heading-dependent
   footprint as a fixed lateral interval and removed the solver's ability to trade heading against
   lateral position.
3. Two linearized `e_y/e_psi` half-spaces per stage. A single local gradient did not conservatively
   enclose the nonlinear rectangular footprint over the QP heading range, and 40 additional rows
   materially increased overruns.

All three changed approximation or timing without establishing a stronger physical proof. They were
deleted rather than hidden behind another feature flag. Exact-pose post-solve certification remains
the truthful boundary until a conservative nonlinear/convex footprint formulation is designed.

## Swept-path boundary

`SweptPathViolation` means discrete predicted poses pass but the segment from the current measured
pose fails. It is not repaired by loosening or contracting later stage bounds. It belongs to a
dedicated first-stage reachability slice.

## Telemetry

Outcome transitions now include lateral violation, permitted tolerance, normalized violation, and
stage. Aggregate certification and unchanged `authority=shadow, selected=0` continue to expose the
physical result without affecting production commands.
