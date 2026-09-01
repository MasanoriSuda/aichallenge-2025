# Requirements

## Objective

Remove the proven generalization regression caused by giving every teacher or
normal run equal total sample probability regardless of duration.

## Constraints

- retain the old sequence mode for reproducibility
- the new sample mode visits every stored sample and remains deterministic
- compute class weights from the same distribution used by the sampler
- retain material class weighting and every short corrective run
- data splits, representation, loss, gates and production remain unchanged

## Definition of Done

- sample and sequence modes have explicit manifests
- class-mass calculation follows the selected sampler contract
- one projected spatial-plus-speed candidate is evaluated at fixed gates
- no runtime promotion occurs on any failed gate
