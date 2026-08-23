# Design: certified warm-start publication

## Root cause

`solve_extended_progress_problem()` currently writes every OSQP-successful
primal/dual and its progress origin into `last_solution` before callers run the
execution-primal normalizer, wall/obstacle certificate and canonical command
chain. If a later check rejects that artifact, the rejected raw primal still
seeds the next solve. Its progress origin is also updated independently of
certification.

The observed signature is deterministic: every production execution-primal
reject in the bounded run used `warm=1`; clearing two dual boundary rows did
not remove it.

## Lifecycle correction

Introduce a `CertifiedWarmStartStore` containing the primal/dual, certification
time and progress origin as one artifact.

1. A solve consumes the previous certified artifact exactly once.
2. Raw solver success is returned but not stored.
3. The caller performs semantic normalization and its physical/canonical
   checks.
4. Only an accepted caller publishes the normalized primal plus solver dual.
5. Any rejected descendant leaves the store empty, forcing a genuine cold
   next solve rather than replaying rejected evidence.

This is an authority-publication correction, not a cold-retry fallback.

## Publication points

- Track/Cruise: complete fresh canonical command and physical certificate.
- Follow: complete worker canonical plan after effective-gap and wall proof.
- Overtake: complete fresh canonical selection after physical proof.
- Left/right tactical branch: physically executable, finite branch evaluation.

## Non-goals

- Solver row preconditioning.
- Warm-start stage-time resampling.
- Retained-plan expansion.
- Parameter tuning.
