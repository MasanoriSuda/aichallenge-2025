# Requirements

## Objective

Reproduce the final representation difference between the admitted diagnostic
probe and the continuous spatial adapter: a frozen seeded projection from the
full conv5 map to 128 dimensions.

## Constraints

- the projection is deterministic, non-trainable and part of strict state
- train-only statistics are computed after projection
- candidate3, data splits, loss, sampling and gates remain unchanged
- train and evaluate exactly one offline candidate
- no runtime or ROS input changes

## Definition of Done

- projection identity changes with seed and is reproducible with the same seed
- evaluation reconstructs the same dimension and seed before strict load
- all gate evidence is recorded whether accepted or rejected
