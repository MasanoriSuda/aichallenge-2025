# Recurrent adapter requirements

## Objective

Test whether temporal dynamic-obstacle corrections become learnable when the
admitted TinyLidarNet course-following representation is preserved instead of
relearned from the small corrective dataset.

## Invariants

- The production checkpoint and runtime authority remain frozen.
- At initialization the candidate steering is numerically identical to the
  frozen base for every finite scan and speed.
- Frozen base parameters are loaded with the strict checkpoint contract and are
  not optimized.
- The adapter may use only current LiDAR history and synchronized ego speed.
- Train/validation run identities and the unseen seed-2028 gate remain intact.
- No runtime integration occurs before every offline gate passes.

## Definition of Done

1. A zero-correction recurrent adapter proves base-output identity.
2. Checkpoint serialization remains self-contained and strict.
3. The candidate is trained on the same recurrent dataset as the rejected
   from-scratch direct policy.
4. Material, anchor, full validation and unseen-run gates are evaluated.
5. Production is unchanged on any gate failure.
