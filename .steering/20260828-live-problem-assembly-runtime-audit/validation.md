# Validation

## Static gates

- `make autoware-build`: PASS (25 packages).
- `colcon test --packages-select multi_purpose_mpc_ros`: PASS.
- `colcon test-result --verbose`: 2027 tests, 0 errors, 0 failures,
  0 skipped.

Only the existing setuptools warnings were observed during the build.

## Dynamic comparison

### Before the ownership correction

Run: `output/20260828-205642`

- decision 1945: total problem assembly 66.060 ms;
- live behavior region: 65.931 ms;
- all other measured assembly regions combined: 0.129 ms;
- following runtime window: 17 callbacks/s, 15 overruns/s, MPC average
  55.330 ms;
- decision 1961: live behavior region 68.479 ms.

### After the ownership correction

Run: `output/20260828-210555`

- start-grid live behavior region: approximately 0.116--0.184 ms;
- the latest-only tactical worker submitted and completed results while the
  production callback consumed them;
- the controller entered canonical `ShiftOut`;
- callback throughput remained approximately 40--41 cycles/s in ordinary
  windows;
- the continuous 17 Hz / 15-overrun collapse did not recur.

The correction therefore removed the synchronous start-grid topology and
Mission generation from the 40 Hz callback without changing production
authority, solver settings, clearance or timing policy.

## Separate finding

The post-fix run exposed a narrower secondary hotspot in
`update_overtake_line()`:

- decision 1968: 22.539 ms;
- decision 2044: 27.429 ms;
- decision 2113: 26.392 ms;
- decision 2153: 27.831 ms;
- decision 2250: 22.258 ms.

These events caused occasional overruns but not the former continuous
callback collapse.  Intermittent canonical Emergency / solver-artifact gaps
also remain during `ShiftOut`.  Both are explicitly out of scope for this
Slice and require a new causal audit; they must not be treated as a reason to
undo the validated scheduling correction.

## Result

PASS for the Slice objective: start-grid tactical generation now has the
same asynchronous ownership as ordinary tactical generation, and the live
control callback no longer duplicates that work.
