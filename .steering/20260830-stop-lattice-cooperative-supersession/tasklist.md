# Tasklist

- [x] Audit the worker, evaluator, mailbox and controller data flow.
- [x] State and compare root-cause hypotheses before implementation.
- [x] Add opt-in worker supersession token without changing legacy semantics.
- [x] Make the Stop observation evaluator exit at safe supersession points.
- [x] Add deterministic worker/evaluator/mailbox tests.
- [x] Extend telemetry and source authority audit.
- [x] Run build, full package tests and `git diff --check`.
- [x] Commit static acceptance before dynamic exercise.
- [ ] Run `make dev2` and classify freshness/tail evidence.
- [x] Observe newer-submission supersession in `output/20260830-230003`.
- [x] Isolate the final-source invalidation gap after Overtake intent exit.
- [x] Invalidate running/pending observation work on certified non-Overtake replacement.
- [ ] Re-run static and dynamic gates for the refined lifecycle contract.
