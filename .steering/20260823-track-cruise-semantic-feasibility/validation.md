# Validation

## Baseline

`output/20260823-075629` completed six laps and produced 97 strict canonical
execution-primal rejects. Classification:

| field / stage | warm | cold | total |
|---|---:|---:|---:|
| curvature / 0 | 75 | 6 | 81 |
| acceleration / 0,1,6 | 10 | 0 | 10 |
| virtual-progress speed / 0 | 4 | 0 | 4 |
| predicted velocity / 1 | 0 | 2 | 2 |

The presence of eight cold events rejects warm-start shifting as the sole root
cause. Curvature saturation is the dominant manifestation.

## Failure-first result

Before implementation, `test_persistent_osqp` failed to compile because
`SolverConfiguration` and `SolveTelemetry::polishing_enabled` did not exist.

The proposed two-variable synthetic mixed-unit QP did not reproduce the real
horizon miss: default OSQP reached normalized row violation `2.27e-13`. The
test was corrected rather than forcing an artificial failure. It now proves:

- default `PersistentOsqpSolver` is polish-off;
- explicit dedicated construction is polish-on;
- both results still pass the independent rowwise report.

Focused persistent-OSQP tests: 9/9 passed.

## Static gates

- `make autoware-build`: 25 packages passed.
- full `multi_purpose_mpc_ros` test: 1,633 tests, zero error/failure/skip.
- Existing unrelated `joycon_contract_guard/package.xml` stale-build warning
  remains in `colcon test-result` and does not affect the package result.

## Dynamic gate result

`output/20260823-083410` is not an acceptance run. The first non-interactive
`make dev` launch observed `spawned` and one initial certified
`polish=1` candidate, but AWSIM did not progress to Grounded/Ready/Start and
odometry stopped. Domain 0 had no admin-start subscriber. The run was stopped
without a lap and must not be used for performance or reject-rate comparison.

`make eval` could not start because local image
`aichallenge-2025-eval:latest` is absent. Building a new evaluation image was
not introduced as part of this controller Slice.

The subsequent normal `make dev` run `output/20260823-084006` reached Start and
completed lap 1 in 46.336 s. The experiment failed its primary acceptance
criterion:

| execution-primal reject | warm | cold | total |
|---|---:|---:|---:|
| curvature, stage 0 | 23 | 0 | 23 |
| acceleration, stage 0 | 1 | 0 | 1 |
| predicted velocity, stage 1 | 0 | 2 | 2 |
| virtual progress speed, stage 1 | 1 | 0 | 1 |
| **total** | **25** | **2** | **27** |

For comparison, the observer-enabled one-lap control
`output/20260823-081219` contained 26 execution-primal rejects. The change is
therefore not material. All 27 rejected outcomes reported `polish=1`, proving
that attribution was correct. Three physical-certificate rejects also
occurred, but they are outside this semantic-feasibility hypothesis.

No callback overrun was reported, but the observed callback maximum reached
23.503 ms against a 25 ms period; one solve window reached 23.179 ms total.
This gives no performance reserve for accepting a correction that did not
improve feasibility.

## Decision

Candidate C is rejected and its code was removed. It did not eliminate the
semantic rejects, did not distinguish warm from cold failures and consumed
most of the control-period budget in its slowest window.

The next Slice must design feasible-primal restoration as a new certified
candidate. It must preserve the raw solver result for warm-start and
provenance, reconstruct dynamics from measured state and bounded inputs, and
pass all semantic and physical checks before authority admission.
