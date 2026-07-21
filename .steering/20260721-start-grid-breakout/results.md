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
- Post-grace-lifecycle correction: 577 tests, 0 errors, 0 failures, 0 skipped.
- Post-open-corridor-priority correction: 580 tests, 0 errors, 0 failures, 0 skipped.
- Post-duplicate-front-cap correction: 580 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.
- A stale unrelated `build/joycon_contract_guard/package.xml` warning was emitted by
  `colcon test-result`; it did not affect the selected package results.

## Third runtime finding and correction

Run `20260721-221420` exposed two independent lifecycle failures. D1 first observed the stopped
front target before the other kart entered `side vehicle`, so breakout eligibility was false and
it changed directly to `SafetyBrake` at 2.65 m. D2 entered `start-grid breakout`, reached `Pass`,
then the controller erased its target and side exactly when the 5.0 s grace expired. D1 also lost
its locked pass at WP63-64 when the rotating hairpin frame hid the selected gap for roughly one
second and the 1.0 s hold expired about 0.03 s too early.

The entry now needs the stationary front classification but not a simultaneous side
classification; the inflated gap planner remains the execution authority for all received
vehicles. Grace expiration no longer clears a same-target active ShiftOut/Pass, while an inactive
or different line cannot reuse the latch. The locked-side transient gap hold is 2.0 s for the
measured hairpin dropout; EmergencyBrake, forbidden WP, cooldown, wall bounds, and target jumps
remain outside that hold.

## Fourth runtime finding and correction

Run `20260721-222751` entered breakout on both d1 and d2, proving the front-only entry and grace
lifecycle changes were active. D1 nevertheless chose left and logged the right side as
`start-grid breakout opposite staggered side`; the right corridor was never evaluated because the
initial grid offset had been treated as a mandatory pass direction. D1 then repeatedly logged
`OvertakeLine: ShiftOut -> Idle, reason=safety brake` even while V2X debug still reported
`desired=Overtake`, `grid_breakout=1`, and `gap=1`. OvertakeLine reapplied the EmergencyBrake risk
metric that breakout arbitration had intentionally bypassed.

An unlocked breakout now evaluates both inflated-vehicle corridors and prefers the larger actual
gap width, with nearest-front geometry used only for a tie. The selected side remains locked after
entry. A validated breakout OvertakeLine is also preserved through the close-front risk metric;
explicit SafetyBrake, an unavailable corridor, blocked zone, and existing execution fail-safes
still cancel it.

## Fifth runtime finding and correction

Run `20260721-224214` proved that both d1 and d2 selected an available corridor and entered
`start-grid breakout` about 0.05 seconds after their first V2X detection. The remaining failure was
longitudinal: d2's behavior released its 37 km/h start reference, but OvertakeLine continued to
report `cap_release=0` and `v_ref=3.42 m/s`, which was the locked target speed plus the generic
0.5 m/s early-Pass allowance. D1 showed the same duplicate cap. Both karts therefore stayed near
the front kart's speed until the right corridor disappeared at WP65-66, exhausted the 2.0 second
gap-loss hold, and returned to `Follow -> SafetyBrake`.

For a still-validated breakout, OvertakeLine now marks its generic front cap released and leaves
longitudinal reference ownership to the dedicated breakout behavior. It continues to own the
lateral target. If the gap or execution zone becomes invalid, the breakout validation becomes
false and the normal cap/fail-safe path is restored.

## Runtime acceptance

Run `make dev3` and check P2 immediately after `AWSIM Start`:

- Expected transition: `None -> Overtake` with reason containing `start-grid breakout`.
- Expected debug: `grace=1`, `grid_breakout=1`, and a valid left or right gap.
- Expected OvertakeLine debug while that corridor remains valid: `cap_release=1`; `v_ref` must not
  remain locked to front speed plus 0.5 m/s.
- Fail-closed expectation: if both side gaps are invalid, reason contains
  `start-grid breakout unavailable` and the state remains `SafetyBrake`.
