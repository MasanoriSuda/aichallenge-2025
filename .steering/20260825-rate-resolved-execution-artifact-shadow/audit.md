# Audit

## Finding

`CanonicalExecutionPlan::CanonicalControlStage` stores curvature, and
`extract_canonical_actuation()` publishes steering reconstructed as
`atan(wheelbase * curvature)`. That is correct for the current five-state
production formulation but is not the solved variable of the six-state
rate-resolved formulation.

The rate-resolved shadow previously retained only the first controls, terminal
state and one publication sample. The complete primal was discarded after the
worker returned. Therefore it could prove numerical viability, but it could
not prove that a future retained execution path would preserve the exact
six-state solution.

## Root cause statement

Production promotion is blocked because the new formulation has no immutable
execution representation matching its state/input semantics. Reusing the old
representation would create a second lateral actuator owner at extraction and
would invalidate the solver-to-publisher proof.

## Chosen correction

Add a separate, shadow-only six-state artifact first. Do not expand the old
plan with optional fields and do not switch authority in the same Slice.
