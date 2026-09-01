# Tasklist

- [x] Freeze the capacity-only hypothesis and comparison boundaries.
- [x] Train the 512-unit capacity candidate.
- [x] Prove manifest equality except hidden dimension and output identity.
- [x] Evaluate seed 2033, unseen seed 2035 and production-normal behavior.
- [x] Compare against the 64-unit peer run and previous admitted candidate.
- [x] Do not convert: interaction-material non-regression is not demonstrated.

## Decision

Capacity is a real factor: the 512-unit candidate removes the broad regression
of the 64-unit peer model.  It is not the complete answer.  Material MAE is
0.73% worse than the previous candidate on seed 2033 and 0.73% better on seed
2035.  This is parity, not demonstrated improvement on both fixed worlds, so
the candidate remains offline and unconverted.
