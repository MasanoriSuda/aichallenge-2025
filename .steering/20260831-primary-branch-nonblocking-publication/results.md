# Results: nonblocking primary normal branch publication

## Root-cause result

The frozen D1 Follow snapshot from `output/20260831-031354` classified as
`A fails, B succeeds`: the persistent pipeline could not build its terminal
successor while both stateless left and right Bundles were certified by the
same seven-state SQP. Production inspection found that even a fully certified
preferred branch was withheld behind `std::async` sibling completion.

This is a producer scheduling/lifecycle defect. It is not evidence for a
clearance, solver tolerance, timeout, grace, lease, speed, or fallback change.

## Implemented invariant

- Normal avoidance uses one persistent bounded sibling executor.
- The homotopy-owner preferred branch is evaluated as primary by the
  latest-only normal producer.
- A fully certified primary is inserted into the candidate Store immediately;
  sibling completion is not awaited.
- The data-only branch bank accepts independently completed sides only under
  the exact immutable source epoch. A new epoch invalidates both old sides and
  a late older sibling is rejected.
- If primary certification fails, a completed certified sibling may replace
  the candidate. Completion order alone cannot replace a certified primary.

## Static verification

- `make autoware-build`: 25 packages completed.
- Focused branch-bank and single-authority tests: 2/2 passed.
- Full package CTest: 59/59 passed (45 gtest, 12 pytest-labelled tests).
- `git diff --check`: passed.
- Host `pre-commit` remains unavailable.

## Dynamic acceptance

Run: `output/20260831-033922` (`make dev3`).

Observed in D1 and D2:

- 15 aggregated Track/Cruise windows ended with a certified primary and
  `store=accepted` (D1 10, D2 5).
- Certified primaries were admitted both when `sibling_submit=accepted` and
  when `sibling_submit=busy`.
- D1 contains explicit examples where the optional worker was busy but the
  primary still logged `primary_bank=accepted/store=accepted`.
- Latest sampled executor counters showed no job exception: D1
  `submitted=149/completed=148/failed=0`; D2
  `submitted=78/completed=78/failed=0`.
- Branch-bank telemetry reported `invalid_source=0` and `invalid_plan=0`.
  Nonzero `stale` counts are expected late old-epoch sibling rejections, not
  adoption; the bank never regressed its source sequence.
- The control callback does not wait on the sibling executor. D1 primary
  publication continued while the executor reported busy; D2 still has
  unrelated callback/recovery/publisher overruns which this Slice does not
  claim to fix.

## Residual failures kept separate

The run is not a race-quality acceptance:

- D1 later entered prolonged Follow rejection with
  `initial-hard-gap-violation` after a front-risk Emergency event.
- D3 had no comparable normal dynamic-avoidance primary episode and retained
  separate Stop/override interruptions.
- Overtake planner infeasibility and SafetyBrake transitions remain.
- The bounded sibling worker is frequently busy. This is now localized to
  optional evidence and no longer withholds a certified primary, but sibling
  coverage/throughput remains measurable future work.
- No six-lap result JSON was produced by this short dynamic run.

Those observations do not invalidate the completed Slice, and they do not
authorize a new timer, lease, fallback, safety relaxation, or parameter patch.
Each requires its own frozen failure snapshot and architecture comparison.
