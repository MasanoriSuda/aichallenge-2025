# Task list: proof-guided SQP acceptance

- [x] Freeze production authority and parameters.
- [x] Make audit SQP depth explicit and bounded.
- [x] Add per-candidate proof-guided depth evaluation.
- [x] Add focused CLI/report mode.
- [x] Add tests for depth-0 short-circuit and audit isolation.
- [x] Replay both frozen counterexamples.
- [x] Replay accepted regression snapshots.
- [x] Decide promotion or next root cause.
- [x] Build and document.
- [ ] Commit.

## Verification

- `test_mpcc_rate_resolved_dynamic_obstacle`: 19 passed
- `test_mpcc_stateless_maneuver`: 17 passed
- `test_mpcc_rate_resolved_shadow`: 39 passed
- `test_mpcc_architecture_comparison`: 14 passed
- `make autoware-build`: 25 packages passed
- Only the existing `setuptools` deprecation warning was emitted.

## Frozen replay result

- Nine ShiftOut snapshots from seven runs were compared with production
  authority unchanged.
- Two previously uncertified sides became certified at depth 1:
  `20260829-012053/d1/...1612` left and
  `20260829-065458/d2/...5244` right.
- Every side already certified by the single-SQP baseline remained certified
  at depth 0.  The proof-guided arm did not execute a later iterate for those
  candidates.
- No previously certified side regressed in this corpus.
- Several solver-infeasible candidates remained infeasible.  The mechanism is
  therefore not a generic fallback and does not hide candidate-generation or
  physical-infeasibility failures.

## Decision

The audit supports a separate production-promotion Slice: replace one-shot
candidate acceptance with proof-guided acceptance for the same sealed
candidate and proof chain.  Promotion still requires an execution-time budget
and must remove the replaced one-shot acceptance branch in the same Slice.
This observation-only Slice does not alter authority.
