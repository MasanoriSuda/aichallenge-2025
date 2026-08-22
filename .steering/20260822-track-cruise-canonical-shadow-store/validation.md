# Validation

## Baseline reproduction

At baseline `d5f6aec`, source inspection finds the canonical plan adapter and store only in their
libraries/tests. `mpc_controller_cpp.cpp` has no reference to either contract. Therefore a
physically certified shadow solve cannot leave an executable full-plan snapshot.

## Required runtime evidence

The periodic shadow line must expose:

- canonical extraction count;
- canonical store acceptance count;
- last extraction/store reason;
- `authority=shadow, selected=0`.

For every final `status=certified`, extraction and storage must both be accepted.

## Implemented flow

- The shadow evaluator constructs `CertifiedMpccSolution` only after the lateral constraint contract
  and swept world-frame wall certificate both pass.
- The complete extended primal and exact stage durations are sent to the direct canonical adapter.
- Store replacement failure becomes `canonical-store-reject`; it cannot be reported as certified.
- Legacy conversion is optional comparison telemetry and no longer gates canonical certification.
- Control-history reset clears the visible snapshot while preserving the store's monotonic ID
  high-water.
- The store is not read anywhere in command selection or publication. Telemetry remains
  `authority=shadow, selected=0`.

## Verification

- Canonical adapter/store/cursor focused test: passed, including exact second-stage input recovery.
- `make autoware-build`: 25 packages passed.
- Complete package CTest: 35/35 passed.
- `colcon test-result --verbose`: 1595 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

`colcon test-result` still prints the pre-existing stale
`build/joycon_contract_guard/package.xml` discovery warning. It does not create a test failure.

## Dynamic validation status

No AWSIM or Autoware container was running. Starting the simulator would cross the external GUI
operation boundary, so this Slice does not claim dynamic evidence. On the next user-started trial,
the periodic `Track/Cruise MPCC shadow` line must show `canonical extracted/stored` equal to the
number of physically certified plans; any mismatch now carries an explicit reason.
