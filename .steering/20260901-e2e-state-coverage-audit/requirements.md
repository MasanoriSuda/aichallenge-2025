# Requirements

## Objective

Determine whether frozen candidate3's NPC and peer failures are caused by
missing state coverage, single-frame observation aliasing, or a policy error in
already covered states before authorizing another model experiment.

## Constraints

- production checkpoint and `fixed_lidar_brake` remain unchanged
- use only immutable bags and admitted run-level datasets
- compare physical scan geometry and the frozen model's learned embedding
- keep teacher output diagnostic; it does not become production authority
- do not infer success from offline nearest-neighbour metrics

## Definition of Done

- NPC and peer failure windows have explicit immutable provenance
- cross-run nearest-neighbour baselines exclude the source sequence itself
- teacher correction demand and local action ambiguity are reported separately
- the next Slice follows from evidence rather than a threshold adjustment
