# Results: late exact-disjunction candidate

## Root-cause classification

The frozen decision-2970 dynamic-obstacle and coupled-wall snapshots produced
the same result. The old production direct, mid and late coupled-diagonal
candidates did not produce a certified artifact. An offline exact-disjunction
arm did, using the unchanged seven-state SQP and unchanged exact proofs:

- side: right;
- stay-behind rows: stages 0--16;
- selected-side rows: stages 17--19;
- first-ahead stage: 20, outside the finite horizon;
- candidate fingerprint: `8757027856347829556`;
- terminal progress: 6.77068 m;
- terminal velocity: 1.35623 m/s;
- minimum lateral reserve: 1.34242 m;
- continuation depth: zero.

This classifies the failure as candidate generation. The persistent-Mission
lifecycle had already been removed from this comparison, a direct single SQP
was sufficient, and the result passed the existing physical certificate.

## Implemented change

The bounded population still has at most three members per selected side. The
third member was replaced atomically:

- deleted: `LatePhysicalDiagonal` and its last-third coupled half-space;
- added: `LateExactDisjunction`, with a three-stage complete selected-side
  suffix and complete stay-behind rows before it;
- unchanged: worker, Store, authority, publisher, solver settings, clearance,
  weights, timeout, lease, retry and fallback.

The optimizer keeps the selected-side soft reference, so lateral motion may
start before stage 17. Stage 17 is when the complete selected-side physical
disjunct becomes mandatory, not a request to delay steering until stage 17.

## Static and frozen acceptance

- `make autoware-build`: passed, 25 packages.
- focused `test_mpcc_stateless_maneuver`: passed, 19/19.
- package CTest: passed, 54/54.
- source-contract pytest: passed, 75/75.
- both frozen decision-2970 snapshots: the new production-right member passed
  the unchanged SQP, swept-wall, timed dynamic-obstacle and terminal-successor
  proofs. The infeasible opposite side remained fail-closed.

## Dynamic acceptance

Run: `output/20260829-235457`, Domain 1.

The replacement crossed the complete production path twice:

| Source | Rows | Exact proof | Production |
|---|---|---|---|
| artifact 3371 / source decision 3981 | behind 17, side 3, diagonal 0 | physical accepted; dynamic clear, 3.77367 m | decision 3996, `emergency=0` |
| artifact 7648 / source decision 8679 | behind 17, side 3, diagonal 0 | physical accepted; dynamic clear, 5.28752 m | decision 8694, `emergency=0` |

The adjacent callback windows after each publication reported zero overrun.
The second artifact was preceded by a separate aggregate callback window with
four overruns, and the complete run still contains callback tails. Because the
candidate count did not increase and the solve remains on the existing async
worker, this run does not prove a candidate-specific timing regression, but it
also does not close the system timing Gate.

The run is longer and contains different encounters than baseline
`output/20260829-230250`; the lower observed Emergency ratio is supporting
evidence only, not a controlled performance comparison. No Pass completion was
observed, so this Slice is accepted only as a candidate-generation correction,
not as evidence that overall Overtake quality is complete.

## Separate residuals

The same run exposes two independent failure families that this Slice does not
patch:

1. upstream legacy Mission wall gates still emit
   `dynamic Mission wait has no wall-feasible lateral authority` and enter
   Recovery even while the canonical producer can later certify a ShiftOut;
2. an old published artifact can later fail retained current-world admission
   with `progress-lift-rejected` after about 2.14 s, for example decision 10008
   with progress delta -1.54657 m versus the unchanged 1.5 m proof bound.

These are lifecycle/ownership and scheduling evidence for later frozen
snapshots. They do not justify changing the proof bound, adding a grace period,
or weakening wall constraints in this Slice.

## Decision

Accept the atomic candidate replacement. Keep the exact proof chain and
bounded population unchanged. Next work must freeze and classify the retained
progress-lift / legacy Mission-gate residual instead of tuning this candidate.
