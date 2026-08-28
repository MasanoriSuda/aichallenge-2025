# Design

The serialized exact QP contains the original racing Hessian and all physical
and artificial refinement rows.  The lag bucket rows are identity state-box
rows at:

`state_values + stage * 7 + lag_index`

Relaxing only those bounds reconstructs the audit problem without changing
matrix expressions, racing weights or physical wall/opponent rows.

An offline Python tool solves the same physical QP through several numerical
representations.  Every returned solution is unscaled before objective and
constraint residual evaluation.  A small C++ audit-only external-primal mode
may ignore only the selected artificial bucket during affine residual checking;
it then uses the existing physical adapter and exact proof chain unchanged.

This is not solver fallback design.  It is an architecture escape-hatch audit
whose result must select one formulation/backend or reject the branch.
