# Signed-mixture residual requirements

## Objective

Test whether explicitly separating negative, anchor and positive steering
intent removes the opposing-homotopy averaging observed in the binary-gate
residual.

## Invariants

- This Slice is offline-only until every admission gate passes.
- Production checkpoints, launch defaults and runtime NumPy inference do not
  change.
- The frozen base checkpoint continues to define normal steering.
- Direction classes and magnitudes are learned only from LiDAR-derived model
  input and admitted teacher labels.
- An uncertain negative/positive decision must cancel toward zero instead of
  choosing an arbitrary large correction.
- Train and validation remain seed-disjoint.

## Definition of Done

1. Exact zero residual before training.
2. Explicit negative/anchor/positive class supervision with balanced weights.
3. Separate negative and positive magnitude heads.
4. Offline evaluation covers both hard tails, seed 2027, unseen seed 2028,
   historical anchors and independent normal data.
5. Runtime work starts only if all gates pass.
