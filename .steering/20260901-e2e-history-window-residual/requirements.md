# History-window residual requirements

## Objective

Determine whether a bounded LiDAR history makes pre-contact left / anchor /
right corrections observable without changing the production controller.

## Invariants

- Production checkpoint, runtime, launch defaults and fixed LiDAR brake remain
  frozen.
- Use the same seed-disjoint v4 data and signed-mixture head as the rejected
  two-frame comparison.
- History is rebuilt within each recorded run and never crosses a sequence
  boundary.
- Initial frames are padded with the first real frame, not zeros from an
  invented world state.
- No runtime work or closed-loop run is allowed unless every offline admission
  gate passes.
- Do not tune admission thresholds to accept a model.

## Definition of Done

1. An explicit eight-frame contract is covered by tests.
2. The comparison changes only temporal observation length.
3. Both hard tails, full train/validation runs, historical anchors and an
   independent normal run are evaluated.
4. A failed model is rejected with a causal classification and no runtime
   wiring.
