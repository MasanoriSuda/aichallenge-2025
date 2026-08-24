# Validation

## Root cause addressed

The existing warm-start helper presented a state/input-generic API while its
dual layout always inserted one legacy curvature-rate row per stage. A
six-state formulation whose physical actuator is `delta_dot` already owns the
rate through its input box and therefore has no such row. Reusing the hidden
five-state layout made an otherwise valid six-state warm start fail its exact
dimension contract.

The helper now accepts an explicit `MpcWarmStartLayout`. The established API
delegates with `rate_rows_per_stage=1`, so production behavior is unchanged;
the rate-resolved QP declares zero duplicate rate rows. Tests verify the exact
stage-by-stage primal and dual transport rather than merely accepting vector
sizes.

## Commands and results

- `make autoware-build`
  - PASS: 25 packages.
- Targeted CTest in `/aichallenge/workspace/build/multi_purpose_mpc_ros`
  - PASS: `test_mpcc_rate_resolved_problem`.
  - PASS: `test_persistent_osqp`.
- Full `colcon test --packages-select multi_purpose_mpc_ros`
  - PASS: 42 CTest targets, 1,814 tests, zero errors/failures/skips.
  - The existing stale `build/joycon_contract_guard/package.xml` result-parser
    warning remains unrelated; the final test summary is clean.
- Production link audit
  - PASS: `mpc_controller_cpp` does not link either rate-resolved shadow
    library.
- `git diff --check`
  - PASS.

## Scope boundary

This Slice does not alter runtime control, parameters, authority, fallbacks or
published commands. It establishes a solvable, row-decodable numerical
contract for the next controller-side shadow adapter. No dynamic trial is
required because no production target links the new implementation.

## Remaining work

Construct the rate-resolved problem from the existing controller observation
and stage geometry in shadow, solve it asynchronously, and compare feasibility,
runtime, wall proof and 40 Hz actuation publishability with the production
five-state formulation. Authority remains prohibited until that evidence and
fresh/retained artifact lifecycle exist for the new schema.
