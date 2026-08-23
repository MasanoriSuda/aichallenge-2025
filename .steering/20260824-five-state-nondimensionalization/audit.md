# Audit

## Observed failure chain

`output/20260824-022828` showed OSQP success followed by five-state
execution-primal rejection. Metre-scale progress inflated one global stopping
tolerance while acceleration, velocity and curvature-rate rows required
millimetre-scale physical accuracy.

The first variable-scaled run (`output/20260824-025157`) removed the large
reject population but retained alternating stage-0 input failures. Strengthening
all row scales (`output/20260824-025957`) still rejected acceleration and
virtual-progress bounds. Joined diagnostics in `output/20260824-030745` proved
that OSQP's reported residual equalled the exact transformed-problem residual;
the mismatch was between solver preprocessing and the physical certificate.

## Root cause

The physical certificate evaluates tolerance against the boundary actually
crossed. Row preprocessing instead used the larger absolute value of both
bounds. For acceleration `[-3, 1.37]`, an upper violation was solved with the
`-3` tolerance. For virtual progress `[0, about 11]`, a lower violation was
solved with the distant upper tolerance. OSQP's global relative stopping term
then applied relative tolerance a second time across all transformed rows.

This was a producer-contract defect, not warm-start transport, wall geometry,
an execution clamp, or an OSQP residual-report defect.

## Repair

- Five-state variables are explicitly transformed with `x = D z`, with `D`
  derived from the formulation's finite box bounds.
- Each finite row uses the strictest finite-side physical tolerance.
- That tolerance maps to OSQP's absolute tolerance.
- The row-normalized solver disables its global relative term because physical
  relative tolerance is already embedded per row.
- Primal and dual warm starts and returned solutions are transformed in both
  directions; final certification remains in original physical coordinates.
- Track/Cruise now uses the same row contract as Follow and Overtake.

No vehicle parameter, retry, fallback, timeout, lease, flag, clamp or alternate
normal authority was added.

## Deleted masks

The five-state Track/Cruise `None` preconditioning path is gone. A five-state
solve can no longer pass a global mixed-unit admission while relying on a later
partial execution-box detector to reject it. Legacy three-state authority is
unchanged and remains migration debt for a later deletion Slice.

## Residual risk

The bounded run exercised Track/Cruise and Follow but did not produce a live
Overtake interval. Overtake uses the same solver adapter and tests pass, but its
live Gate must be repeated before that authority is promoted further.
