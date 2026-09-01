# Requirements

## Objective

Explain the first failure in the frozen packaged four-vehicle E2E Gate before
changing production authority.  Distinguish a model decision defect from a
runtime composition, speed-input, longitudinal-safety or post-contact symptom.

## Frozen evidence

- baseline commit: `667f733c`
- run: `output/20260902-e2e-final-packaged`
- packaged spatial authority SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- base checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

## Constraints

- Do not change the production checkpoint, steering authority, pace cap or
  LiDAR safety distances during diagnosis.
- Do not use the longest low-speed interval as the failure precursor without
  proving it is the first sustained stop.
- Keep bag replay explicitly counterfactual; it is not closed-loop proof.
- Preserve all ROS 2 and submission interfaces.

## Definition of Done

- base, learned residual, composed steering and published steering are
  comparable on the exact same scan windows;
- the earliest post-motion stop is used rather than a later recorder segment;
- failed d1/d2 are compared with clean d3/d4 at the same course region;
- one bounded A/B is selected that can falsify the remaining lateral-policy
  hypothesis without changing production defaults.
