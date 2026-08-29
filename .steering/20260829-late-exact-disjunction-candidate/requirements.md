# Requirements: late exact-disjunction candidate

## Frozen baseline

- Baseline: `c302bf31`
- Dynamic run: `output/20260829-230250`, Domain 1
- Frozen failure family: decision 2970 ShiftOut dynamic-obstacle and coupled
  wall-refinement solve rejection

## Expected and actual behavior

Expected: the bounded current-world production population contains a feasible
same-side trajectory whenever the unchanged seven-state SQP and exact physical
proof chain can certify one in the current immutable world.

Actual: direct, mid physical-diagonal and late physical-diagonal production
candidates fail.  The architecture comparison finds one certified right-side
candidate when it keeps the complete behind disjunct through stage 16 and uses
the complete side disjunct for stages 17--19.  It is accepted directly by the
same single SQP; offline continuation is not used.

The result reproduces in both the dynamic-obstacle and coupled-wall snapshots.

## Root cause

The late production member forces a coupled diagonal separating half-space
from stages 13--18.  That fixed geometric transition removes a feasible region
that still exists when the candidate selects only complete behind/side
disjuncts and lets the MPCC dynamics and soft lateral reference choose the
actual transition trajectory.

This is a candidate-generation defect, not a Mission lifecycle defect, solver
tolerance issue or general physical infeasibility.

## Constraints

- Do not change solver settings, clearance, proof tolerance, authority,
  publisher, timeout, lease, grace, retry or fallback.
- Keep the production population bounded to three candidates per side.
- Replace the old late candidate atomically; do not append another candidate.
- Rebuild every candidate from one immutable current-world fingerprint.
- Preserve the exact wall, timed obstacle and terminal-successor proof chain.

## Definition of done

- A 20-stage encounter produces a late exact-disjunction candidate with side
  stages 17--19 and no ahead row inside the current horizon.
- The old `LatePhysicalDiagonal` production kind and construction are deleted.
- Both frozen failure snapshots certify the replacement with unchanged proofs.
- Focused tests, source contracts, package tests and build pass.
- A bounded dynamic run shows no stale/uncertified publication and improves the
  alternating ShiftOut-to-Emergency failure family without callback overrun.
