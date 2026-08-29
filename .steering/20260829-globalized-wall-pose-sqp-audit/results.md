# Results: globalized wall-pose SQP audit

## Observation

The first target was the frozen ShiftOut wall-refinement failure with source
fingerprint `11478535197026802675`.  Omitting either the heading bucket or the
lag bucket alone still ended in an OSQP maximum-iteration rejection.  Omitting
both artificial pose buckets together solved in `116.248 ms` and passed the
unchanged exact trajectory, swept wall, timed current-world obstacle and
terminal-successor proofs.

The certified bundle reached `14.875 m` terminal progress and `7.07755 m/s`
terminal velocity.  The arm has no Store, mailbox, publisher or production
authority connection.

## Frozen-corpus replay

The same observation-only arm was replayed on six ShiftOut wall failures:

| Source fingerprint | Joint-pose result |
| --- | --- |
| `11478535197026802675` | certified bundle |
| `1467348131628347049` | exact lateral proof rejected after the existing bounded corrections |
| `11833690901213757627` | coupled racing QP numerical rejection |
| `7444296560492909208` | certified bundle |
| `12904723043417630620` | certified bundle |
| `5403542176957632812` | certified bundle |

Thus the change recovered four of six previously rejected artifacts while
the unchanged proof chain rejected the unsafe trajectory and the unresolved
numerical case.  It is not a blanket acceptance path.

The previously analysed dynamic counterexample with fingerprint
`12107242968934788374` was also replayed.  Its joint-pose QP solved, but the
timed physical obstacle proof rejected a new overlap.  Removing the pose boxes
therefore cannot bypass current-world obstacle evidence.

## Root cause

The lag and heading intervals are local pose-validity regions for one sampled
wall bucket.  Applying both as hard state boxes after the MPCC has already
bound progress and lateral wall rows can empty the first affine subproblem,
even when an exact physically certified trajectory exists.  The failure is a
trust-region/globalization defect at the wall-refinement boundary, not a
clearance, Mission lifetime, timeout or generic solver-tolerance problem.

The exact proof chain is the authoritative certificate.  The pose boxes are
only an approximation aid and must not independently veto an artifact that
can be checked against the immutable physical world.

## Decision

A separate production Slice is justified: replace the hard post-hoc lag and
heading wall pose boxes with proof-authoritative physical acceptance while
retaining lateral/progress rows, exact nonlinear rollout, swept-wall proof,
timed obstacle proof and terminal successor proof.  This is a replacement of
one formulation boundary, not an added fallback.

The production Slice must delete the old hard pose-box authority in the same
change.  It must retain rejection when the exact proof fails and must not add
clearance, tolerance, iteration, timeout, lease or resume-rule changes.

One corpus member still has a numerical racing-QP rejection.  That case is
evidence for a later feasible-QP/globalized-SQP solver Slice, not a reason to
keep the known-empty pose-box intersection for every candidate.
