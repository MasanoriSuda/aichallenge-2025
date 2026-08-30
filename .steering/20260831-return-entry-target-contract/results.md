# Results: post-solve dynamic proof ownership

## Observed failure

In `output/20260831-072258/d1`, Pass sequence 936 remained executable until
decision 1688. Return drafts did not acquire authority and decision 1690 fell
to Emergency Stop, followed by wall Recovery. Replaying decision 1690 itself
found no feasible A/B/C/D arm, so changing Return target handling there would
have patched a downstream physical failure.

The earlier sequence 942 was the causal boundary. Live dual evaluation solved
and passed exact wall proof, but emitted
`exact_dynamic_final=invalid/blocked/obstacle=` and certified neither branch.
The same immutable snapshot certified both bounded production sides offline.

## Root cause

The exact dynamic proof compared two values with different lifetimes and
owners:

- `ReplayWorld::bound_tolerance_m`: `1e-5 m`, captured before solve;
- final physical snapshot tolerance: derived after solve from the accepted
  artifact's actual maximum constraint violation.

Any legitimate accepted residual above the baseline made the two unequal.
Dynamic proof then rejected source provenance before checking one obstacle.
This was a scheduling/lifecycle-looking failure caused by a model/certificate
ownership mismatch.

## Implemented correction

- ReplayWorld no longer owns the post-solve certificate tolerance.
- Exact trajectory and final wall snapshot must carry one identical
  artifact-derived tolerance.
- Offline architecture comparison now constructs the same final certificate
  pair as production.
- Dynamic source validation reports a typed reason such as
  `certificate-tolerance-mismatch`, rather than only an empty obstacle id.

No authority, solver, Mission, timeout, lease, fallback, clearance, weight or
configuration changed.

## Verification

- `make autoware-build`: 25 packages succeeded.
- focused dynamic-proof and architecture tests passed.
- complete package suite: 59/59 CTest targets, 2301 tests, zero failures.
- frozen sequence 942: production-left and production-right both produced
  certified ManeuverBundles with the unchanged proof chain.
- bounded `make dev3`: `output/20260831-074836`.

Dynamic results in the bounded run:

| Domain | valid/clear | valid/blocked | invalid | selected dual certified |
|---|---:|---:|---:|---:|
| D1 | 28 | 2 | 0 | 8 |
| D2 | 14 | 0 | 0 | 6 |
| D3 | 0 | 0 | 0 | 0 |

D2 sequence 1031 certified both sides and stored the selected ShiftOut branch.
This directly demonstrates that the former live `invalid` defect is removed.

## Remaining independent failures

This Slice does not claim Overtake quality completion. D1 later left Pass due
to `SafeSeparation aborted: local time limit`; D1/D2 also observed actual wall
margin failures, stale/lost targets and Recovery. D1 had ten dual evaluations
where the selected side was rejected while its sibling was certified. Those
are distinct tactical/lifecycle families and require new frozen snapshots;
they must not be hidden by tolerance or Return-target changes in this Slice.
