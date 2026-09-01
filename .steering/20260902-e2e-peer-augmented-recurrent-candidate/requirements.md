# Requirements

## Objective

Train and evaluate one peer-augmented projected-conv5 recurrent candidate while
holding architecture, frozen baselines and training hyperparameters equal to
the previous admitted offline candidate.

## Constraints

- Change the certified training corpus only; do not tune model, loss, deadband
  or runtime parameters in the same experiment.
- Freeze raw TinyLidarNet and production spatial-v11 tensor identities.
- Train on seed 2034 plus the four final-world peer sequences.
- Select and evaluate on independent seed 2033 and independent production-
  normal validation data.
- Do not use the successful peer source as validation after training on it.
- Do not package, launch or grant authority to the candidate in this Slice.

## Definition of Done

- Training manifest proves the same model family and immutable baselines.
- Candidate passes finite-output, embedded-identity, held-out teacher and
  normal-leakage gates.
- Metrics are compared directly with the previous recurrent candidate.
- A runtime shadow experiment is proposed only if offline evidence is not
  worse and materially relevant.
