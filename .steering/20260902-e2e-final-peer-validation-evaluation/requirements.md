# Requirements

## Objective

Use the admitted independent final-peer rollout as a validation-only corpus to
compare the frozen production recurrent model with the already trained peer
augmentation candidates.

## Constraints

- Preserve the run-level split: every sequence from the new rollout is `val`.
- Require the embedded executed-teacher success certificate for every source.
- Keep the explicit 0.100 s causal speed freshness used by the executed teacher.
- Do not add these sequences to any training root.
- Do not retrain, convert, package or connect a candidate in this Slice.
- Compare only immutable, already produced checkpoints.
- Record that this is execution-disjoint but not world-disjoint evidence.

## Definition of Done

- Every sequence that satisfies the executed-teacher and causal freshness
  contracts is extracted; a failing domain is excluded rather than repaired by
  relaxing the contract.
- A certified recurrent validation-only derivative is built.
- Production, frozen peer-64, peer-512 and run-balanced candidates are evaluated
  with the same decode and freshness contract.
- Candidate decisions are made from material, anchor and overall errors without
  changing runtime authority.
