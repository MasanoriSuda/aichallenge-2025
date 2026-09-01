# Design

Use the existing projected-conv5 recurrent training pipeline unchanged.  The
only experimental variable is the recurrent dataset root:
`speed_committed_recurrent_peer_augmented_v1` replaces
`speed_committed_recurrent_v1`.

Keep:

- `frozen_tinylidar_adapter`;
- projected conv5 dimension and seed;
- fixed train statistics;
- no scalar speed input;
- the frozen production spatial-v11 baseline;
- the independent production-normal recurrent corpus;
- 0.02 rad material/deployment deadband;
- optimizer, loss weights, epochs and sequence sampling mode.

The four peer sequences are one correlated training world.  Their successful
outcome makes them admissible labels, not independent validation.  Seed 2033
and production-normal validation remain the offline decision boundary.

The final-world teacher used the runtime's 0.100 s causal speed freshness
contract.  The recurrent loader previously hard-coded 0.050 s.  Keep 0.050 s
as the fail-closed default, but require an explicit training/evaluation option
to admit the runtime-matched 0.100 s corpus and persist that option in the
manifest/report.

Before evaluating a trained model, compare its manifest with the previous
candidate.  A candidate whose architecture or optimization settings differ is
not evidence for the dataset experiment, even if its metrics improve.
