# Requirements

## Objective

Remove the observed conflict between normal activation suppression and
left/right correction generalization by factorizing those decisions.

## Constraints

- keep the same frozen base, projected representation and real speed contract
- keep sample sampling, data splits, loss weights and gate thresholds
- initialize the composed correction to exact zero
- store all factorized head state in the strict candidate artifact
- train and evaluate one offline candidate only

## Definition of Done

- neutral activation and conditional sign are separate outputs
- their composed three-class probabilities remain compatible with existing loss
- the correction is finite, bounded and exactly zero before training
- no candidate reaches runtime unless every existing gate passes
