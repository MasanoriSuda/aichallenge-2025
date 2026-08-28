# Design

Physical wall refinement samples an exact footprint around a provisional
trajectory and then emits hard lateral, lag, heading and progress state boxes.
Independent LP decomposition showed that the frozen QP becomes feasible when
either the heading or lag boxes are omitted, even while the actual wall rows
remain.

Add an audit mode at the wall-refinement construction boundary. Production
uses `EnforceAll`. The comparison tool can request `OmitHeading` or `OmitLag`.
Only the selected artificial state bucket is omitted; the resulting trajectory
must still pass the existing nonlinear physical wall and dynamic-obstacle
certificates before an audit bundle is reported.

This is deliberately not a tolerance change. It tests whether a local proof
bucket is unnecessarily acting as an execution constraint.

The audit is two-stage.  It first replaces the racing objective with a strictly
convex identity projection while retaining every selected hard row.  If that
Phase-I problem solves, the exact original racing objective is restored and
solved from the feasible primal with a cold dual.  Only the second solution may
become an audit bundle, and it must still pass all unchanged exact physical
proofs.  This prevents numerical feasibility evidence from being mistaken for
a performant production trajectory.
