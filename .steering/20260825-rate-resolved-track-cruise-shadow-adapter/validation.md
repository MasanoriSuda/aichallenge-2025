# Validation

## Structural result

The adapter now has one explicit conversion from the established semantic
five-state Track/Cruise snapshot to the rate-resolved QP request. Curvature is
not retained as a second actuator:

- desired curvature and its box become steering-state reference and box;
- curvature tracking weight is converted with the local `d(kappa)/d(delta)`;
- adjacent-curvature weight becomes steering-rate magnitude weight through
  `delta_dot * dt`;
- steering rate has no duplicate delta cost or constraint row.

Stage zero initially exposed a known historical risk: a supplied nominal state
could differ from the hard observed state. The final adapter makes the
observation snapshot the sole owner of stage-zero reference and the first
linearization anchor, and rejects an observation outside its state-zero box.

## Validation

- `make autoware-build`
  - PASS: 25 packages.
- `test_mpcc_rate_resolved_adapter`
  - PASS: exact mapping, stage-zero ownership, curved OSQP solve and malformed
    input rejection.
- Full `colcon test --packages-select multi_purpose_mpc_ros`
  - PASS: 43 CTest targets, 1,819 tests, zero errors/failures/skips.
  - Existing unrelated `joycon_contract_guard/package.xml` result-parser
    warning remains.
- Production link audit
  - PASS: `mpc_controller_cpp` links none of the rate-resolved libraries.
- `git diff --check`
  - PASS.

## Dynamic evidence

Not applicable. The adapter remains test-only and cannot affect a published
command. The next Slice is the explicit runtime boundary: populate this
snapshot from the existing Track/Cruise builder and execute it through an
isolated latest-only shadow worker with identity, age and timing telemetry.
