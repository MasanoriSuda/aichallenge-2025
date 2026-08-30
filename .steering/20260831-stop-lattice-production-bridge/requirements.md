# Requirements

## Objective

Remove the missing source-replacement edge proven by run
`output/20260830-235803`: when ordinary ShiftOut/Pass authority is unavailable,
an accepted Stop lattice plan that passes the unchanged current-world
seven-state contract shall remain normal authority instead of immediately
publishing external emergency Stop.

## Root-cause evidence

At decision 1689 ordinary source 1053 failed because its terminal Stop wall
proof collided.  Stop lattice source 990 passed the same current-world
evaluator (`joined=1`, `reason=accepted`), but was observation-only and was
discarded.  Production published 0 m/s external Stop and latched it on later
cycles.

## Constraints

- Do not add a Mission resume rule, lease, grace, timeout, retry or fallback.
- Do not change solver tolerance, wall clearance, candidate population,
  candidate ordering or control weights.
- Evaluate the lattice alternate at most once per control cycle.
- Preserve one canonical normal publisher and the existing immutable identity
  and current-world proof contracts.
- Prefer ordinary authority whenever it is valid.
- Select the lattice alternate only for ShiftOut/Pass and only after ordinary
  authority fails.
- Keep external emergency Stop when ordinary, lattice and existing published
  Stop successor sources all fail.
- Remove the observation-only duplicate join edge in the same Slice.

## Definition of Done

- Failure-first ownership tests encode selection order and single evaluation.
- The accepted lattice result is selected without a second current-world
  evaluation.
- No new Store, publisher or legacy authority owner is introduced.
- Build, package tests and source-contract tests pass.
- `make dev2` proves at least one lattice production bridge or proves that no
  joinable lattice source existed at the observed ordinary loss.
- Later wall-margin Recovery remains separately visible and is not masked by
  a clearance change.
