# Temporal steering residual requirements

## Objective

Test whether one-step LiDAR motion context separates the opposite lateral
corrections that the frozen single-frame residual averaged to zero.

## Invariants

- Production TinyLidarNet, fixed LiDAR braking and default launch behavior do
  not change.
- Runtime lateral output remains ML: frozen base plus learned ML residual.
- No V2X, GNSS, map, trajectory or teacher decision enters student inference.
- The temporal feature uses only current LiDAR and the immediately preceding
  LiDAR scan from the same runtime/recorded sequence.
- Train/validation sequence identity and all existing residual target
  provenance checks remain mandatory.
- A temporal checkpoint must be explicitly identified; stateless and temporal
  shapes may never be guessed or partially loaded.

## Definition of Done

1. Dataset and runtime construct identical `current, current-previous` input.
2. Torch and NumPy temporal models agree within the existing numeric tolerance.
3. First-frame behavior is deterministic and finite.
4. The candidate improves both opposite hard-state tails without exceeding
   normal/anchor leakage gates.
5. Runtime A/B is attempted only after offline gates pass.
