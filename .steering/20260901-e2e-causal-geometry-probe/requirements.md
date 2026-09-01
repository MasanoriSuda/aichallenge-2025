# Requirements

## Objective

Determine whether causal spatiotemporal LiDAR geometry resolves the remaining
teacher/normal action ambiguity after all static representations failed.

## Constraints

- production v11 and runtime remain frozen;
- diagnostic left/neutral/right classification only;
- physical LiDAR, synchronized wheel speed and frozen-base steering are the
  only inputs;
- history resets at every immutable sequence boundary;
- train/validation and normal/teacher run identities remain disjoint;
- use the current v6 teacher and current production-normal corpus;
- three seeds, peer/focus/final-200 metrics and normal leakage are mandatory;
- do not export or integrate a runtime checkpoint.

## Definition of Done

- a causal local-geometry encoder plus recurrent state is implemented and
  tested for boundary correctness;
- three deterministic diagnostic runs complete;
- comparison with the frozen static baseline is recorded;
- failure either closes static/temporal model search and returns to label/data
  redesign, or justifies one offline continuous residual experiment.
