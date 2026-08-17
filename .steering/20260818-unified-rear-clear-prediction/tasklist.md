# Task list

- [x] Compare the latest run with the preceding equal-duration run.
- [x] Identify the two inconsistent rear-clear rollout paths.
- [x] Add a pure runtime-to-horizon rear-clear resolver and tests.
- [x] Remove the duplicate controller rollout and use the shared result.
- [x] Run focused tests and the Autoware package build.
- [x] Review and commit without the user's local changes.
- [ ] Dynamically validate with an unchanged `make dev2` setup.

## Verification

- `colcon build --packages-select multi_purpose_mpc_ros --symlink-install
  --cmake-args -DBUILD_TESTING=ON`: passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 28 CTest entries
  passed.
- `colcon test-result --verbose`: 1261 tests, 0 errors, 0 failures,
  0 skipped.
