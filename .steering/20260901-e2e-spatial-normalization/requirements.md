# Requirements

## Objective

Test whether the continuous spatial adapter failed because its per-sample
LayerNorm did not match the train-feature standardization used by the admitted
separability probe.

## Constraints

- candidate3 remains frozen and byte-identical
- use the existing recurrent teacher and synchronized normal splits
- compute statistics from train sequences only
- store the exact mean and scale inside the candidate artifact
- do not change loss weights, gate thresholds or runtime authority
- run one offline candidate before deciding on further architecture work

## Definition of Done

- fixed statistics have strict shape, finite and positive-scale checks
- evaluation reconstructs the exact normalization contract before loading
- embedded base identity and all existing gates remain active
- rejected candidates never reach runtime
