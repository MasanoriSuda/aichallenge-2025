# Validation

## Baseline dynamic run

- Run: `output/20260822-225811`
- Source: `2dd1d33f3cb0ecd08e8725cec80e4c4f839cbbdb`
- Scenario: one-car `make dev`, five observed waypoint wraps
- Result: Gate A failed
- Reproduced: 324 `canonical-plan-reject/invalid-control-stage` outcomes
- Separating evidence: solved/finite/physical certificates remained valid, inspected callback
  windows had zero overrun, and authority remained shadow-only.

## Failure-first result

Before implementation, `test_mpcc_progress` failed to link because
`normalize_extended_execution_primal()` did not exist. The tests therefore identify the newly
required numerical/semantic contract rather than merely exercising an existing path.

After implementation:

- targeted `test_mpcc_progress`: 58/58 passed;
- `make autoware-build`: 25 packages passed;
- package test: 35/35 CTest targets, 1,576 tests, zero error/failure/skip.

## Post-fix dynamic Gate A

- Run: `output/20260822-232351`
- Scenario: one-car `make dev`, five observed waypoint wraps
- Summarized eligible/solved cycles: 9,795/9,795
- `invalid-control-stage`: 0 (baseline: 324)
- Certified chain:
  `physical=canonical stored=cursor=candidate=fresh=actuation=9,678`
- `actuation_diff`: 0 in every reported window
- Within-row-tolerance normalization: 1,492 cycles / 2,857 values,
  maximum adjustment `0.000997522 m/s`
- Exact semantic row rejection: 3 cycles
- Physical checks/certificates: 9,792/9,678; the remaining physical failures preserve the already
  documented legacy-created current-pose/first-stage provenance and are not relaxed here.
- Callback: 2 overruns in 256 one-second windows, maximum 28.138 ms. Shadow maximum was 13.541 ms,
  so the observed callback maximum is not explained by shadow solve time alone.

The three semantic rejections were all stage-zero virtual-progress lower-bound violations:

| decision | raw value [m/s] | row violation | row tolerance |
|---:|---:|---:|---:|
| 4639 | -0.00330833 | 0.00330833 | 0.00100331 |
| 7799 | -0.00136516 | 0.00136516 | 0.00100137 |
| 8002 | -0.00174945 | 0.00174945 | 0.00100175 |

They were previously accepted by `PersistentOsqpSolver` because it computes per-row normalized
residuals but makes the final decision using one mixed-unit global absolute tolerance. The new
boundary correctly rejects them instead of hiding them with a clamp. This is the next upstream
root-cause Slice and still blocks authority promotion.

## Final root-cause audit

- No config, OSQP setting, wall margin, cost or horizon changed.
- No fallback, feature flag, timeout or authority path was added.
- Raw primal remains the solver/warm-start artifact.
- One separate normalized primal is the only downstream executable artifact.
- Canonical validation remains strict and production remains legacy-owned.
- The baseline defect is closed; the newly exposed rowwise solver-admission defect is not hidden
  or folded into this Slice.
