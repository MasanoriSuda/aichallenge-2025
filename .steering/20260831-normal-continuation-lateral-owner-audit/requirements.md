# Requirements

## Objective

Classify the recurring normal Cruise continuation loss after terminal Stop
wall ownership was corrected. Determine whether the first
`initial-lateral-bound-rejected` / continuation `invalid-lateral-bounds`
failure is caused by execution lifecycle, candidate generation, single-SQP
approximation, wall-model mismatch or physical infeasibility.

## Frozen evidence

- Dynamic run: `output/20260831-043038`.
- D3 first retained authority loss: decision 2770, sequence 2169,
  `continuation=model:initial-lateral-bound-rejected`.
- Nearest complete immutable Cruise snapshot:
  `d3/mpcc_architecture_snapshots/000000002177-c92a4b03b33d1d6b-`
  `cruise-side-neutral-wall-refinement-solve-rejected/snapshot.yaml`.
- The generic A/B/C/D comparison cannot classify target-free Track/Cruise;
  every arm reports `selected current-world target unavailable`.

## Constraints

- Production authority remains unchanged during classification.
- Do not change solver tolerances, weights, wall margins, clearance, leases,
  grace periods, timeouts or fallback behavior.
- Do not infer physical infeasibility from a missing-target audit arm.
- Use immutable same-snapshot evidence and exact downstream proof.
