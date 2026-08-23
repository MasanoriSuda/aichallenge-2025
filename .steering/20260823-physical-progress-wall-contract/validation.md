# Validation

## Failure-first evidence

After adding tests that required a physical-progress-dependent corridor API,
the package rebuild failed in `build_2026-08-23_10-05-56` because that API did
not exist.  This confirmed that the tests were not passing against a stale
binary and that the previous implementation could not express the required
contract.

The tests cover:

- a state accepted by the old fixed-stage bound but rejected at solved
  physical progress;
- identical progress coupling for lag and virtual progress;
- fail-closed handling of malformed or non-monotonic corridor provenance;
- the typed extended constraint layout, including two corridor rows per
  predicted stage;
- warm-start shifting of the two additional stage-wise dual blocks.

## Static validation

- Build: `build_2026-08-23_10-29-39` succeeded.
- Targeted tests:
  - `MpccProgress`: 63/63 passed.
  - `PersistentOsqpWarmStart`: 9/9 passed.
- Full `multi_purpose_mpc_ros` package test: 38/38 CTest entries passed.
- Aggregate `colcon test-result`: 1637 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

The unrelated user-owned modification in `aichallenge/result-summary.json`
was not edited by this Slice and must not be staged.

## Dynamic acceptance

The candidate was run as `make dev` in `output/20260823-103351` and compared
with `output/20260823-081219`.

| Metric | Baseline | Candidate |
|---|---:|---:|
| eligible cycles | 3321 | 3083 |
| solved cycles | 3313 | 3068 |
| execution-primal accepted | 3154 | 2934 |
| physically certified | 3154 | 2934 |
| logged solve-failure outcomes | 1 | 5 |
| callback overruns | 0 | 3 |

The candidate did keep physical wall-certificate reject counters at zero, but
the additional coupled rows increased numerical load at tight curvature
bounds.  The run showed repeated curvature `certified-bound-violation`, OSQP
maximum-iteration/solve failures, emergency-stop authority, and a callback
window with three overruns.  This violates the explicit acceptance criteria;
the candidate is rejected rather than rescued by solver, tolerance, margin or
weight tuning.

## Disposition

All source and test changes from this candidate were removed with
`apply_patch`.  Only this causal audit remains.  The result does not invalidate
the confirmed coordinate mismatch; it shows that adding two affine rows per
stage to the current monolithic QP is not an acceptable repair under the
existing real-time and semantic-bound contracts.

The next Slice must not repeat this formulation.  It should first isolate the
physical-progress corridor proof from the existing actuator-bound
normalization/authority path, and preserve the full five-state solution
provenance through Overtake execution instead of converting it to a three-state
layout before certification.

The legacy-layout adapter remains downstream of the five-state solve in part
of the Overtake execution path.  It is not being hidden with a fallback or
parameter change.  It is the next structural boundary to audit before another
physical-corridor formulation is attempted.
