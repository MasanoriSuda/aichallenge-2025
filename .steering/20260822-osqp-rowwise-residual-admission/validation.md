# Validation and rejected result

## Baseline

- Run: `output/20260822-232351`
- Five waypoint wraps
- `invalid-control-stage`: 0
- `execution-primal-reject`: 3
- Observation: each rejected virtual-progress value exceeded the canonical adapter's exact
  semantic row tolerance.  This proved that those three results cannot become executable canonical
  plans; it did not yet prove that the shared solver must reject every such local-row report.

## Failure-first and static result

- Before implementation, the target failed to link on the absent
  `constraint_residuals_satisfied()` predicate.
- Experimental target tests: 9/9 passed.
- `make autoware-build`: 25 packages passed.
- Full package test: 35/35 CTest targets, 1,577 tests, zero errors/failures/skips.

Static tests alone therefore did not detect the integration regression.

## Dynamic counterexample

- Run: `output/20260822-234326`
- Scenario: clean one-car `make dev`
- Earliest rejection: first legacy normal solve after initial pose
- Failure:
  `stage=constraint_check, row=166, violation=0.0144887, tolerance=0.0103059,
  normalized=1.40587, status=solved inaccurate`
- Row identity: first curvature-rate constraint of the legacy 3-state/2-input QP
- Propagation: shared workspace reset -> repeated cold `maximum iterations reached` -> forced-stop
  fallback -> zero-speed vehicle and more than 1,600 solver failures
- Result: dynamic regression; experiment rejected before a lap was attempted

## Final audit

- Experimental header/source/test edits were removed.
- No parameter, solver setting, flag, timeout, fallback or authority was changed.
- `aichallenge/result-summary.json` was not touched.
- The three five-state semantic misses remain rejected before certification.  The next structural
  step is retained same-formulation revalidation, which is designed to bridge a transient fresh
  miss without returning to legacy MPC.
