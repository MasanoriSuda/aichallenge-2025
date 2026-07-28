# Task list

- [x] Confirm the latest cap-release to SafetyBrake sequence.
- [x] Reconcile candidate and execution corridor geometry.
- [x] Implement staged Pass clearance.
- [x] Align committed-line wall clearance.
- [x] Add boundary regression tests.
- [x] Run package tests and `make autoware-build`.
- [x] Record runtime confirmation points.

## Validation

- `make autoware-build`: success, 25 packages.
- `multi_purpose_mpc_ros`: 24/24 CTest entries passed.
- `colcon test-result`: 686 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: clean.

`colcon test-result --verbose` reported the existing missing
`build/joycon_contract_guard/package.xml` artifact while scanning unrelated build results; it did
not affect the selected package and the result summary remained clean.

## Runtime confirmation

In the next `make dev2` run, verify:

- startup reports wall clearance `0.10 m`, front-cap clearance `1.50/1.50 m`, and body clearance
  `1.45 m`;
- `body_clear=1, lat_clear=0` can occur in a committed Pass without `Pass -> Idle`;
- in that zone `cap_release=0` and closing speed is limited to the configured Pass value;
- `cap_release=1` returns after lateral separation recovers to at least `1.50 m`;
- actual separation below `1.45 m`, another vehicle, wall contact, or physical infeasibility can
  still invoke existing protection;
- P1 completes `Pass -> Return` without collision or wall penalty.
