# Requirements

## Objective

Eliminate the static spatial candidate's independent-normal correction leakage
by admitting train-only production states as exact zero-residual anchors.

## Constraints

- retain the spatial architecture, optimizer, loss and every Gate threshold
- ignore normal corpus control labels; their target is correction zero
- restore physical metres before passing scans to the adapter
- preserve sequence identity and sequence-balanced sampling
- keep `dagger_aggregate_v2/val`, teacher validation and peer d3 untouched
- no runtime or shadow authority

## Definition of Done

- normal anchor provenance appears in the training manifest
- no train/validation identity overlap
- material improvement, direction and peer Gates remain passing
- independent normal MAE decreases from 0.01939 rad to at most 0.01 rad
- production candidate3 and launch defaults remain bit-for-bit unchanged
