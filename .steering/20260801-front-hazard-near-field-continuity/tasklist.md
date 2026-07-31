# Task List

- [x] Confirm the residual event and preserve unrelated user changes.
- [x] Define the bounded near-field continuity policy.
- [x] Add the pure resolver and unit tests.
- [x] Integrate the resolver into V2X behavior classification.
- [x] Run formatting/build/unit tests.
- [x] Record verification results and remaining dynamic validation.

## Static Verification

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: succeeded.
- `colcon test-result --verbose`: 752 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: succeeded.
- Host `clang-format` was unavailable; the package's configured lint/test targets passed in Docker.

## Dynamic Verification

Run `make dev2` and confirm:

- A stopped/slow target cannot transition SafetyBrake -> Cruise while it remains within 4.0 m
  and inside the inflated lateral danger band.
- `front hazard hold refreshed by near-field target` appears only at the front/side/rear geometry
  seam, not throughout a laterally clear pass.
- The prior collision episode does not recur, and solver failure/stuck recovery counts decrease.
