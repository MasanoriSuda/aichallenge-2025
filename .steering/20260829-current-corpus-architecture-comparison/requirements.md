# Requirements: current corpus architecture comparison

## Objective

Classify the replay-ready ShiftOut failures after causal suffix reconstruction
failed to rescue them. Compare persistent A, stateless B, rough/lattice C and
offline continuation D on the same immutable world before any production
change.

## Constraints

- keep production authority and controller parameters frozen;
- use the existing seven-state model, physical proofs and solver policy;
- do not infer physical infeasibility from optimizer failure;
- compare at least one dynamic-obstacle and one post-refinement failure before
  generalizing to the corpus;
- inspect current primary literature or established implementations if all
  bounded in-repository arms fail.

## Exit

Open an implementation Slice only when the same-world evidence distinguishes
Mission lifecycle, candidate generation, single-SQP/nonlinear basin, physical
infeasibility or model/certificate mismatch.
