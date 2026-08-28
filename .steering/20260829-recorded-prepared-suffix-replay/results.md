# Results: recorded prepared-suffix replay

## Corpus

The replay-ready v2 corpus contains seven 20-stage ShiftOut failures:

- two dynamic-obstacle-refinement failures;
- one post-refinement-linearization failure;
- two wall-refinement-coupled failures; and
- two wall-refinement failures.

There is no replay-ready Pass snapshot. This Slice therefore does not claim
Pass coverage and does not infer production safety from ShiftOut evidence.

## Same-QP time-aligned feedback result

Each snapshot was replayed at `0.025 s` and `0.25 s`. The latter consumes one
stage and rebuilds a 19-stage suffix. In all fourteen runs:

- the semantic suffix and final prepared QP rebuilt successfully;
- construction cost was approximately `0.89--1.23 ms`;
- the cold feedback solve reached OSQP status `-2` after `4000` iterations;
- feedback solve cost was approximately `35.8--41.6 ms`; and
- the full semantic pipeline also returned `solve-rejected`, costing roughly
  `118--238 ms`.

The transform is cheap enough to remain an architectural option. It is not a
repair for these frozen failures: preserving the recorded refined problem and
moving it onto a causal stage suffix leaves every failure infeasible to the
current solver.

## Root-cause classification

The evidence falsifies the narrow hypothesis that these failures are caused
only by joining a prepared QP at an old execution time. A causal connector is
still required, but connector timing is not sufficient to recover this corpus.

The current classification is:

- scheduling/lifecycle-only defect: not supported by these seven failures;
- candidate generation or single-SQP limitation: still open and now higher
  priority;
- physical infeasibility: not proven;
- model/certificate mismatch: not established because no replay solve
  succeeded and then failed proof.

No production authority, solver tolerance, wall clearance, timeout, lease,
fallback or controller parameter changed.

## Live-shadow decision

Do not add a runtime shadow yet. A runtime connector would add scheduling and
telemetry surface without evidence that the preserved prepared QP can produce
a usable result. The next bounded experiment must compare alternate candidate
families and multi-step/nonlinear feasibility on the same immutable failures.

## Verification

- `make autoware-build`: 25 packages passed.
- complete `multi_purpose_mpc_ros` CTest: 54/54 targets passed.
- aggregate result: 2100 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
