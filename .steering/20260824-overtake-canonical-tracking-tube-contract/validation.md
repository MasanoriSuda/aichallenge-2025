# Validation

## Static gates

- `git diff --check`: passed.
- Single-authority source contract: 16/16 passed.
- `make autoware-build`: 25 packages completed successfully.
- Focused CTest targets:
  - `test_mpcc_progress`
  - `test_canonical_execution_plan`
  - `test_canonical_execution_plan_adapter`
  - Result: 3/3 targets passed.
- `colcon test --packages-select multi_purpose_mpc_ros`:
  1,744 tests, zero errors, failures or skips.

The focused tests prove that contraction never silently reduces the requested
reserve, state zero remains governed by the physical corridor, future nominal
states must satisfy the contracted tube, and the reserve is sealed through
canonical extraction.

## Dynamic gates and counterexamples

### State-zero contract correction

Run `output/20260824-142025` rejected every Overtake branch because the first
implementation demanded reserve from the observed state-zero equality. The
contract was corrected to require reserve only from controllable future states
`1..N`.

### Intent lineage correction

Run `output/20260824-143403` exposed pre-entry candidates being solved under a
Cruise-like bounds contract and later sealed as ShiftOut/Pass. QP construction,
bounds schema and plan lifecycle now all derive from the explicit prospective
or current canonical intent supplied before solve.

### Tracking tube accepted, later producer discontinuity exposed

Run `output/20260824-145739` entered ShiftOut and logged stored retained
authority with `tracking_tube=0.150m`. Before failure the expected nominal
state retained at least `0.170m` of physical reserve. Thus the plan did not
consume the configured tracking reserve.

Fresh candidate supply then stopped while the tactical worker repeatedly
reported `no complete or receding branch candidate`. A roughly `1.75s` old
stored plan remained in use; measured lateral state drifted about `0.186m`
from expected and crossed the physical corridor by `0.015m`. Current-world
proof rejected it and the explicit Emergency owner published `-3.0m/s2`.

## Conclusion

The Slice closes the tracking-tube contract defect without changing parameters,
tolerances, leases, fallback, phase transitions or normal authority ownership.
It intentionally does not hide the newly isolated producer defect. The next
Slice must make pre-entry and active ShiftOut/Pass/Return produce a fresh
same-side receding canonical candidate on each accepted update, or explicitly
prove infeasibility. Reusing an old plan longer, weakening physical proof, or
adding another fallback is rejected.

## Unrelated workspace state

`aichallenge/result-summary.json` is user/run-owned and is excluded from this
Slice and its commit.
