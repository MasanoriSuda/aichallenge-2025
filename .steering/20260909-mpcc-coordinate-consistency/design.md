# Representation and repair

The previous fixed-control comparison rejects the current Frenet equations and
also rejects merely adding ideal arc-length rotating-frame terms. A complete
frame-derivative formulation and independent Cartesian integration agree on
the sealed real course. The original model instead changes the native wall
reserve verdict. This is the producer defect, not a reason to weaken the proof.

Compare two complete representations of the same seven-state nonlinear
problem: physical Cartesian x/y/yaw with contour/lag costs composed through
the reference transform, and existing Frenet coordinates obtained by exact
projection of the same Cartesian step. The mapping is invertible for each
fixed progress and preserves nonlinear costs/constraints. Cartesian solver
variables would additionally replace every existing Frenet cost/constraint
Jacobian and their consumers. Prefer retaining the current solver state order
and applying a Cartesian step internally if native replay and QP checks agree.

The transition reconstructs the initial pose on immutable course support,
integrates x/y/yaw with the unchanged acceleration and exact steering-response
ramp, advances only theta by nu, then projects into that same course window.
This includes frame/chord changes without approximate curvature integration.
An explicitly analytic constant-curvature course remains available for unit
geometry; runtime world-backed requests must bind their exact course window.
No missing world geometry may silently select an analytic reference.

Bind geometry through solver input, artifact, fresh/retained/Stop integration
and replay. Preserve source fingerprints and use a new model/state schema to
exclude old artifacts/warm starts. Physical proof and SQP consume the same
transition, with an independent Cartesian oracle in regression tests.

No normal authority is added. The old virtual-frame differential law is removed
in the same slice. Keep existing geometry-domain and actuator validity checks;
evaluate any remaining closest-course progress calculation separately.
