# Design: time-aligned prepared-QP feedback

## Inputs

- immutable `LatestStateFeedbackPreparation`
- latest control-origin timestamp and seven-state observation
- exact serialized previous input
- solver physical tolerance (used only to reconstruct the existing physical
  input envelope)

## Transformation

1. Resolve the semantic time-aligned suffix.
2. Slice all stage-block vectors from the prepared final problem.
3. Slice progress-aligned wall rows by transition stage.
4. Retain swept-wall rows whose transition has not elapsed and renumber them.
5. Retain dynamic-obstacle rows whose constrained future state has not elapsed
   and renumber them.
6. Replace steering/response reference and bounds with the semantic adapter's
   current-clock values.
7. Build a sliced primal from the prepared solution, overwrite x0, then
   relinearize every surviving transition around that primal.
8. Assemble and solve only this reduced feedback QP in later tests.

## Why all surviving transitions are relinearized

The first shortened stage changes its destination state.  Relinearizing the
complete suffix is cheap model algebra and prevents a new seam between the
corrected first transition and old downstream affine equalities.  Physical
wall/opponent rows remain unchanged at their original absolute future stages.

## Authority

The result is numerical evidence only.  Exact current-world physical proof and
an immutable current fingerprint are still required before any later promotion.
