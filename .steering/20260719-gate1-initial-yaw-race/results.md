# Results

## Verification

- `make autoware-build`: 25 packages completed successfully
- `colcon test --packages-select imu_gnss_poser`: 5 tests passed
- `make gate1`: `output/20260719-153044`
- Safety Gate result: `all_passed=true`, `test1.passed=true`

## Runtime evidence

- `/set_initial_pose` was called before GNSS and returned `no GNSS data received yet`
- First GNSS callback published a raceline-aligned initial pose with yaw `2.216 rad`
- EKF trigger succeeded
- Initial controller error was `e_psi=-0.228 rad`; at AWSIM Start it was `0.002 rad`
- MPC reported `solver_failures=0`
- Ego speed rose from `0.00` to `3.16 m/s` before the expected front-vehicle SafetyBrake

The original failed run `output/20260719-151346` initialized with `e_psi=-2.444 rad` and repeatedly
failed OSQP. The initialization-order failure is therefore reproduced by the same early service call,
while the corrected automatic fallback starts the vehicle without solver failure.

## Known unrelated test state

The workspace-wide historical `colcon test-result --verbose` also reports an existing
`multi_purpose_mpc_ros` path-core failure: the user-updated `traj_mincurv.csv` no longer has the
duplicate circular endpoint expected by that test. The new `imu_gnss_poser` test target itself passed.
