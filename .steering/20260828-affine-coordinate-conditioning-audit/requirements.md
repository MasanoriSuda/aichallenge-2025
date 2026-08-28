# Requirements

## Objective

Classify the two affine-feasible canonical seven-state QPs that remain
unsolved after preconditioning ownership was unified.  Test whether the
failure is caused by representing narrow, non-zero-centred physical trust
regions with a scale-only coordinate transform.

## Constraints

- Use frozen exact-QP snapshots only.
- Do not change production authority, physical bounds, objective weights,
  solver tolerances, iteration limits, fallback, lease, grace or timeout.
- Compare mathematically equivalent coordinate transforms on the same QP.
- Do not promote a transform until both the original physical optimum and
  physical-row certificate are shown to be invariant.

## Definition of Done

- The scale-only and affine-centred transforms are reproduced on the same
  snapshots.
- Convergence, physical objective and maximum physical constraint violation
  are recorded.
- The result is classified as coordinate-conditioning, QP-solver, candidate
  generation or physical-infeasibility evidence before production changes.
