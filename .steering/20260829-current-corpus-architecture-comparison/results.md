# Results: current corpus architecture comparison

## Frozen dynamic-obstacle refinement failure

Snapshot:
`000000002970-shiftout-dynamic-obstacle-refinement-solve-rejected`

- Persistent A, stateless B and every rough C member produced no certified
  bundle.
- Offline D found one certified right-side bundle out of 210 schedules.
- The accepted schedule stayed behind until transition stage 17 and reached
  ahead at stage 20.
- The bounded production G population did not contain that late schedule and
  produced no certified bundle.

This is a bounded candidate-population defect for this snapshot. The physical
problem is not generally infeasible because a trajectory passed the unchanged
solver, exact wall, exact dynamic and terminal-successor proofs.

## Frozen post-refinement-linearization failure

Snapshot:
`000000002970-shiftout-post-refinement-linearization-solve-rejected`

The captured persistent right-side Mission failed, but rebuilding candidates
from the same immutable current world produced multiple certified right-side
trajectories:

- offline D: 1 accepted schedule;
- diagonal E: 2 accepted schedules;
- physical diagonal F: 11 accepted schedules;
- bounded production G: accepted `late-physical-diagonal`, stages `13 -> 19`.

The current production candidate family is therefore sufficient for this
world. It is used during pre-entry, but the active normal worker subsequently
solves the retained Mission geometry. This snapshot is a Mission geometry
lifecycle defect, not physical infeasibility and not a clearance/tolerance
problem.

## Common root cause

The two failure families are different, but they share one architectural seam:
the pre-entry current-world candidate population does not remain the geometry
owner after Overtake authority is admitted.

1. Post-refinement failure: an already-implemented bounded current-world
   candidate would have succeeded, but the retained Mission geometry failed.
2. Dynamic-obstacle failure: rebuilding is necessary but the bounded three
   candidates are not temporally rich enough; a much later wait-then-shift
   schedule succeeds.

This explains why adding Mission resume rules or solver tolerances does not
converge. The executable trajectory is retained across a changing world, while
the richer current-world search is confined to entry time.

## Upper-rank comparison

`.steering/ano` shows one continuously running GMPCC plus a separate async
child process and ranked candidate population (`rank=.../3`). The main solved
trajectory continues while candidate work runs. This is consistent with the
derived direction:

- retain tactical identity and the last published certified trajectory;
- rebuild candidate geometry from the current world asynchronously;
- compare a bounded population without blocking command publication.

## Next bounded change

For active ShiftOut/Pass/Return intents, replace the normal worker's direct
solve of retained Mission geometry with the existing same-side bounded
current-world population. This is a source replacement, not a fallback:

- Mission still owns target identity, selected side and commit/no-return;
- candidate paths and phase-transition geometry are rebuilt each submission;
- opposite-side switching is not introduced;
- the existing last-certified retained authority continues during worker
  computation;
- the old retained-geometry solve must become unreachable for Overtake in the
  same Slice.

After this lifecycle replacement is dynamically accepted, expand the bounded
population with a derived late wait-then-shift member only if the dynamic
failure family remains reproducible. Do not combine both hypotheses in one
production change.
