# Coordinate-consistent MPCC completion slice

User authorised execution of the completion plan on 2026-09-09. Continue through
model repair, peer geometry, integrated acceptance and submission evaluation.
This steering owns the first producer repair; previous evidence remains intact.

Invariant: identical physical initial pose and actuator controls must produce
identical Cartesian vehicle motion regardless of virtual progress or the chosen
reference parameterisation. Source6782/failure7405 and the native analytical
counterexamples establish the violation before this change.

Preserve seven states, single normal authority, physical actuator parameters,
hard constraints, margins and topic/submission contracts. Do not retune solver
or timing to hide a failure. Reject old model identities and missing/mismatched
geometry. Stop and retained motion must use the same physical transition.
Keep user's existing result JSON and generated diagnostics outside commits.

Rollback baseline: d54822c9 (full ID recorded with validation). Roll back only
this slice, not earlier committed calibration and Stop repairs.
