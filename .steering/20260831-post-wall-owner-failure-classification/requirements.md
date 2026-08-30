# Requirements

## Objective

Classify the remaining normal-authority failures observed after local physical
wall-bucket ownership was removed from production.  Do not change production
authority until each representative frozen failure has an A/B/C/D exit
classification.

## Evidence source

- Candidate run: `output/20260831-051051` (`make dev3`).
- Production physical bucket hard ownership remained zero on all three cars.
- Historical physical-bucket wall refinement rejection disappeared, but new
  frozen snapshots remain in progress-wall, dynamic-obstacle and terminal
  contingency families.

## Constraints

- No clearance, footprint, solver tolerance, iteration, weight or horizon
  tuning.
- No Mission resume rule, lease, grace, timeout, retry or fallback.
- No publisher or production authority change in this audit Slice.
- Use the same seven-state SQP and exact nonlinear wall/dynamic/Stop proofs.
- Preserve immutable snapshot identity and report unsupported arms honestly.
