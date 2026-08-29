# Validation

## Result

Candidate generation is a real defect, but one reachable candidate plus one
SQP correction is not sufficient evidence for production.

The deterministic 20-stage counterexample has:

- latest state: `heading=-0.35 rad`, `steering=-0.55 rad`,
  `response=-0.44 rad`;
- B/direct time-aligned suffix: numerical solve rejected;
- C/reachable nonlinear rollout: QP solved;
- C exact nonlinear physical adapter: rejected.

This separates two failures which the direct suffix previously conflated:

1. the joined old-state tangent can prevent numerical convergence;
2. a numerically solved first correction can still disagree with exact
   nonlinear execution.

The result matches the predeclared classification `solve succeeds but proof
fails: model/certificate mismatch`. C must remain observation-only. The next
comparison is D: bounded offline multi-SQP of the same immutable problem.

## Invariants checked

- every C candidate stage exactly matches
  `evaluate_temporal_frenet_transition`;
- input box and cumulative steering-prefix intervals are enforced while
  constructing the candidate;
- state reference/lower/upper and input reference/lower/upper are unchanged;
- progress-aligned wall, swept wall and dynamic-obstacle rows are preserved;
- the evaluator exposes no Store, mailbox or publisher API.

## Commands

- `make autoware-build`: 25 packages passed;
- focused reachable-bridge tests: 3/3 passed;
- package regression: 54 targets, 2106 tests, 0 errors, 0 failures, 0 skipped;
- `git diff --check`: passed.
