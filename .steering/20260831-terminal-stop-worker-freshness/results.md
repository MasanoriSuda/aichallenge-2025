# Results

## Observed phenomenon and causal chain

In `output/20260831-102150/d2`, ordinary ShiftOut authority became
`terminal-contingency-unavailable`.  Its fixed track-reference Stop collided
with the wall.  The same immutable snapshot was replayed with the free
seven-state Stop:

- persistent/fixed-path Stop: exact wall contact at stage 37;
- seven-state Stop: solved in about 59 ms, terminal velocity approximately
  zero, exact wall/dynamic proofs accepted, certified Bundle produced.

This classified the original defect as candidate generation, not physical
infeasibility.

The first post-change dynamic run, `output/20260831-105057/d1`, proved that the
seven-state result could now be built and retained (`direct=1:1`) but exposed a
second cause.  A failed direct solve entered the broad control lattice, worker
compute reached about 1209 ms, result age reached 1.29 s and the eventual
current-world join was `steering-unreachable`.

## Implemented changes

- Direct seven-state Stop is attempted before the steering-rate population.
- Production explicitly uses `DirectSevenStateOnly`; the broad lattice remains
  audit-only.
- Latest-only scheduling no longer cancels the running bounded solve.
- Pending observations still coalesce to the newest source.
- Certified results cross producer epochs only inside the same target,
  Mission generation, side and intent.
- Current-world exact revalidation remains mandatory before authority.

No timeout, lease, grace period, resume rule, solver tolerance, clearance,
horizon or control weight changed.

## Static acceptance

- `make autoware-build`: 25 packages passed.
- package suite: 59/59 targets, 2279 tests, 0 errors/failures/skips.
- `git diff --check`: passed.
- A new contract test proves production does not enter the control lattice.

## Dynamic acceptance

Run `output/20260831-110041/d1` exercised one ShiftOut episode.

Before responsibility separation:

- submitted/started/completed: 6/2/2;
- maximum worker compute: about 1209 ms;
- maximum result age: 1.29 s;
- selected direct solve itself: at most about 71 ms.

After responsibility separation:

- submitted/started/completed: 54/51/51;
- population and attempted candidates: exactly 1;
- typical window average compute: 45--71 ms;
- observed maximum direct compute: about 158 ms;
- early result age: about 0.18--0.27 s;
- 51 results were published with no invalid result or decision rollback.

This closes the worker starvation/cancellation defect.  The episode still
failed, so the Slice does not claim Overtake completion.

## Residual failure classification

The residual failure was frozen at decision 2092:

`output/20260831-110041/d1/mpcc_architecture_snapshots/`
`000000002092-907ca51536d2e895-shiftout-side-negative-`
`physical-proof-terminal-contingency-unavailable/snapshot.yaml`

Same-snapshot results:

- A persistent Mission: coupled wall/opponent QP rejected;
- B stateless left: motion solved, but terminal Stop hit the wall;
- B stateless right: dynamic-obstacle QP rejected;
- C rough/lattice/diagonal candidates: no certified Bundle;
- D three-step physical dynamic SQP and proof-guided SQP: no certified Bundle.

The left Stop wall contact occurred around lateral `-2.319 m` at waypoint 64;
the right branch conflicted with the opponent around stages 8--9.  Therefore
all tested arms fail at decision 2092: this is a physically no-escape state at
the observed instant, not evidence for another terminal Stop fallback.

## Existing patches and cleanup

The broad control lattice is retained only because it is useful for offline
candidate-generation audits.  It is no longer allowed to own production
latency.  No legacy command publisher or parallel authority was added.

## Remaining concern and next run

The upstream ShiftOut trajectory progressed from waypoint 37 to about 63
before both the opponent-safe right side and wall-safe left Stop disappeared.
The next Slice must inspect the earliest predecessor snapshot where terminal
successor viability transitions from accepted to unavailable and require
normal candidate admission/continuation to preserve a certified escape.

Next acceptance should verify:

- no normal artifact is published once both homotopies lack a certified Stop
  successor;
- the vehicle exits or stops before the no-escape boundary rather than at it;
- no new lease, timeout, clearance or solver setting is introduced;
- the failure decision moves upstream and remains classifiable by immutable
  snapshot evidence.
