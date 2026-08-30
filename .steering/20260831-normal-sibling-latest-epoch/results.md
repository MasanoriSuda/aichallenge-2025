# Results: latest-epoch normal sibling coverage

## Root-cause result

In `output/20260831-033922/d1`, normal authority disappeared at decision
`1698`, before the physical `SafetyBrake` transition at decision `1705`.
Cruise retention lost its recursive terminal proof while the proposed Follow
intent had no current-world authority. The following hard-gap violation was a
downstream state and was not relaxed.

The frozen decision-1689 Follow snapshot classified as a live
scheduling/lifecycle defect:

- selected live/persistent A: rejected;
- same-side stateless B: rejected;
- opposite-side stateless B with the same seven-state SQP: certified.

The old no-queue sibling executor was running an older epoch and rejected the
current opposite branch as busy.

## Implemented invariant

- The primary normal branch still runs in the existing latest-only normal
  producer and publishes without waiting.
- The sibling now also uses `LatestOnlyWorker`: one running job plus one latest
  pending job.
- A newer pending sibling replaces only an older pending sibling; a running
  solver call is not interrupted.
- The existing coordinator, exact source identity, branch-bank merge,
  certified Store, and current-world production proof remain unchanged.

## Static verification

- `make autoware-build`: 25 packages completed.
- Full package CTest: 59/59 passed (45 gtest, 12 pytest-labelled tests).
- `git diff --check`: passed.

## Dynamic acceptance

Run: `output/20260831-040106` (`make dev3`).

D1 demonstrated the intended scheduling behavior:

- latest sampled sibling worker: submitted 180, replaced pending 20, started
  160, completed 160, exceptions 0;
- a certified primary logged `store=accepted` while sibling submission logged
  `accepted-replaced-pending`;
- same-epoch branch-bank evidence reached `negative=1, positive=1` at source
  sequence 2542;
- branch bank kept `invalid_source=0` and `invalid_plan=0`;
- stale completions were rejected (`stale=69`) rather than adopted;
- no sibling wait was added to the control callback.

## Residual failures kept separate

This short run is not race-quality acceptance. D1 later remained inside a
Follow hard gap, D2 retained Stop after normal terminal proof loss, and D3
entered stuck Recovery after contact/wall proximity. No six-lap result was
produced.

Those failures do not invalidate latest-epoch sibling scheduling and do not
authorize a timeout, lease, grace, fallback, solver, clearance, or speed
change. The next Slice must freeze the first upstream authority loss for one
failure family and run the architecture comparison before implementation.
