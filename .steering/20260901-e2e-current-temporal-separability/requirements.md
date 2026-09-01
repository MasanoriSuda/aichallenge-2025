# Requirements

## Objective

Decide from the current admitted data whether temporal LiDAR evidence can
separate normal no-intervention from material dynamic-obstacle correction.

## Constraints

- freeze production v11 and runtime;
- use current `recurrent_direct_v6_wheel_speed_seed2030` teacher data and
  `production_normal_anchor_v3_wheel_speed_current` normal data;
- compare static and causal temporal representations with identical train
  distribution and deterministic seeds;
- do not train another authority candidate until the representation probe
  supports it;
- do not change labels, Gates or runtime thresholds in this Slice.

## Definition of Done

- static and temporal full-spatial variants are compared over three seeds;
- aggregate, peer, focus, focus-tail and independent-normal classifications are
  recorded;
- the next architecture decision follows from measured temporal gain rather
  than model-name preference.
