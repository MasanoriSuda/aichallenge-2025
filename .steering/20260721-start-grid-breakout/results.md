# Results

## Implemented

- Added `v2x_start_grid_breakout_enabled` and enabled it in the current simulation config.
- Added fail-closed breakout eligibility for the latched stationary grid target.
- Reused the existing inflated-vehicle/wall gap planner for side selection.
- Bypassed the normal 5 m entry and ShiftOut front-speed cap only while the grid breakout is
  active.
- Preserved `SafetyBrake` when no executable side corridor is found.
- Added `grid_breakout` to V2X debug output.

## First runtime finding and correction

Run `20260721-202106` reached the breakout branch but rejected it with
`overtake guard lateral accel, ay=51.1657`. The generic guard measured a move from P2's existing
staggered position to the gap planner's corridor-center target at horizon index 0. That target is
not the bounded OvertakeLine target used for execution.

The first correction required P2 to be established on the selected side by at least the configured
line separation, forbade crossing to the opposite side, and skipped only that mismatched generic
lateral-acceleration estimate. Width, wall, vehicle inflation, consecutive gap, and forbidden-WP
checks remain active.

## Second runtime finding and correction

Run `20260721-202901` removed the lateral-acceleration rejection for d2, which entered
`start-grid breakout`, but changed to `Follow` 0.5 seconds later because the pass side was
recomputed from the changing relative lateral position. In the same run, d1 was rejected with
`start-grid breakout lateral separation` at a 2.60 m front distance and then fell through to the
normal 5 m Follow guard. The implementation had incorrectly reused the 0.75 m OvertakeLine
completion separation as the minimum initial grid stagger and did not latch the chosen side.

Initial side inference now has an independent 0.05 m deadband. A larger observed offset preserves
that side; a nearly aligned pair lets the inflated-geometry gap planner evaluate both sides. Only
invalid/non-finite lateral geometry fails closed. Once a side is selected, it is latched for the
same grid target until breakout reset/expiry, so ShiftOut cannot fall back to Follow only because
the relative lateral value changed. The OvertakeLine completion separation remains 0.75 m and is
unchanged.

## Verification

- `make autoware-build`: passed, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed, 22 test targets.
- Initial implementation: 570 tests, 0 errors, 0 failures, 0 skipped.
- Post-runtime correction: 572 tests, 0 errors, 0 failures, 0 skipped.
- Post-second-runtime correction: 573 tests, 0 errors, 0 failures, 0 skipped.
- Post-side-latch correction: 574 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
- A stale unrelated `build/joycon_contract_guard/package.xml` warning was emitted by
  `colcon test-result`; it did not affect the selected package results.

## Runtime acceptance

Run `make dev3` and check P2 immediately after `AWSIM Start`:

- Expected transition: `None -> Overtake` with reason containing `start-grid breakout`.
- Expected debug: `grace=1`, `grid_breakout=1`, and a valid left or right gap.
- Fail-closed expectation: if both side gaps are invalid, reason contains
  `start-grid breakout unavailable` and the state remains `SafetyBrake`.
