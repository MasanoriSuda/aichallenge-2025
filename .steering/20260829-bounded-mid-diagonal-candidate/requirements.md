# Requirements: bounded mid-horizon diagonal candidate

## Frozen baseline

- Baseline: `c97e8816`
- Dynamic run: `output/20260829-152340`, Domain 1
- Frozen failure: decision 1272, ShiftOut negative homotopy

## Expected and actual behavior

Expected: when a direct side constraint is infeasible, the bounded current-
world production population represents a gradual diagonal transition that the
unchanged seven-state SQP and exact proofs can certify.

Actual: the three production candidates represent direct-side, a two-stage
full-side transition, and a horizon-end transition.  All fail.  With the same
world and solver, the audit lattice first succeeds for a physical diagonal
from stage 0 to stage 9.

## Root cause

The production population samples two extreme transition durations but omits
the ordinary mid-horizon temporal homotopy.  The direct candidate is rejected,
the two-stage candidate asks for an unrealistically abrupt side transition,
and the late candidate postpones separation until the encounter is already
constrained.  Downstream wall and obstacle proofs correctly reject them.

## Constraints

- Do not change solver settings, proof tolerances, wall/opponent clearance,
  authority, leases, timeouts or fallbacks.
- Keep the live candidate population bounded to three.
- Replace an old candidate; do not append a fourth fallback.
- Candidate geometry must be rebuilt from the immutable current world.

## Definition of done

- A 20-stage all-active encounter samples a stage-0 to stage-9 physical
  diagonal instead of stage 0 to stage 2.
- The old two-stage production candidate is deleted.
- The frozen decision-1272 production-right replay becomes certified with the
  unchanged SQP and proofs.
- Unit tests, package tests and full build pass.
- A bounded `make dev2` run shows no stale or uncertified publication.
