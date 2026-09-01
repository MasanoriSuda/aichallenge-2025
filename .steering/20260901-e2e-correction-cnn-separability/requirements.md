# Requirements

## Objective

Test whether a correction-specific trainable 1D convolutional encoder can
separate obstacle-avoidance actions from successful normal driving better than
the frozen lane-following representation.

## Constraints

- production v11 model/runtime remains frozen;
- diagnostic classification only; no steering regression checkpoint;
- input is physical 2D LiDAR, synchronized wheel speed and embedded base
  steering, matching the allowed/runtime observation contract;
- same immutable teacher/normal split, sampling, labels, peer/focus/tail metrics
  and three deterministic seeds as existing probes;
- no teacher threshold, label, Gate or runtime trigger change.

## Definition of Done

- a small local-geometry 1D CNN probe is implemented and tested;
- three seeds compare it with projected and full frozen conv5 baselines;
- normal false-material, peer, focus and failure-tail behavior are reported;
- only strict repeatable improvement justifies an offline authority candidate.
