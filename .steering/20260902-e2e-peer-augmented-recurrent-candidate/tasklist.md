# Tasklist

- [x] Freeze baseline candidate, architecture and data identities.
- [x] Make the runtime-matched speed freshness contract explicit and fail closed.
- [x] Train the peer-augmented candidate with unchanged hyperparameters.
- [x] Evaluate held-out teacher and production-normal behavior.
- [x] Compare against the previous recurrent candidate.
- [x] Reject the candidate; do not convert it to deterministic NumPy.
- [x] Keep runtime authority frozen; a shadow test is not justified.

## Decision

The valid frozen comparison is rejected.  Seed 2033 fails the full-validation
and unseen-not-worse gates, while seed 2035 is worse than the previous
candidate on every policy-error group.  The earlier larger-network run is an
invalid experiment because its architecture and loss weights were not frozen.
