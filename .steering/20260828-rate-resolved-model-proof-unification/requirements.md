# Requirements: Rate-resolved model/proof unification

## Objective

Remove the model mismatch between the seven-state SQP transition and the
exact physical publication proof. A command sequence accepted by the affine
SQP model must be linearized from the same nonlinear stage transition that is
replayed by the publication certificate.

## Frozen evidence

- Baseline: `b6da7ebb`.
- Dynamic run: `output/20260828-014301`.
- Failure snapshot: sequence 890, `physical-proof-rejected`.
- Production exact replay ends 0.216 m below the lateral lower bound.
- A bounded 8-start nonlinear solve using the production 10 ms replay finds a
  feasible command sequence for the same frozen constraints.

## Constraints

- Do not change Mission lifetime, authority, clearance, solver tolerances,
  fallback, timeout, lease, or production intent routing.
- Keep the 10 ms exact nonlinear publication proof fail closed.
- Delete the superseded coarse stage transition rather than retain two models.
- Validate the common transition independently and through the existing
  package suite before dynamic promotion.

## Done

- SQP and publication proof call one canonical nonlinear transition.
- The affine tangent contains the exact canonical reference successor.
- The physical adapter no longer owns a second dynamics implementation.
- Frozen snapshot 890 no longer fails because of a model/proof mismatch, or a
  new recorded failure is classified without tuning.

