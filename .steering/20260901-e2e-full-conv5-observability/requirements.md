# Requirements

## Objective

Determine whether the fixed 1,088-to-128 random projection is the blocker that
merges normal driving with required obstacle-avoidance corrections.

## Constraints

- production v11 and runtime authority remain frozen;
- use the same teacher/normal splits, wheel speed, embedded base steering,
  sample weighting and classifier protocol as the projected baseline;
- compare the complete frozen conv5 map against the 128-dimensional projection;
- retain peer, unseen focus and final-200-sample Gates;
- run three deterministic seeds before considering a trainable candidate;
- do not train or integrate a steering authority in this Slice.

## Definition of Done

- the diagnostic probe supports full conv5 plus speed and base steering;
- cross-label observability is measured before and after projection;
- three seeds establish whether classification and normal leakage improve;
- the result either justifies or rejects a full-map adapter experiment.
