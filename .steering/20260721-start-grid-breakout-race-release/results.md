# Results

## Implementation

- A validated start-grid breakout now releases the full `cfg.v_max` race reference in the entry
  cycle. An unvalidated context retains the previous entry/front+closing cap.
- OvertakeLine stores the first validated ego-center corridor midpoint and reuses it through
  ShiftOut/Pass for both ordinary and start-grid overtakes. The interval already includes target
  inflation and wall clearance. The target-relative goal is retained only as an entry fallback.
- OvertakeLine debug now emits `corridor_goal` for run-time acceptance.
- Explicit-line ownership now suppresses competing lateral targets. Live V2X obstacle bounds keep
  validating ShiftOut/Pass but are not imposed as discontinuous hard MPC state bounds while that
  line owns the maneuver. Before lateral separation is established, a transient infeasible result
  is held for the configured two-second gap-loss window and persistent infeasibility enters
  Recovery. After Pass latches lateral separation, live infeasibility becomes diagnostic-only.
- Before Pass establishes lateral separation, a locked ahead target entering/crossing the
  selected-side common-course ordering cancels ShiftOut/uncommitted Pass. The configured 0.10 m
  margin is an ordering debounce; the gap planner remains responsible for full vehicle inflation.
  After separation is latched, rotating-frame ordering is diagnostic-only and rear-clear owns
  normal completion.
- OvertakeLine supplies both `e_y` and the corresponding Frenet `e_psi` reference.
- An unlocked start-grid entry now uses the front target's ego-relative lateral stagger before
  corridor width when both inflated sides are feasible. It therefore separates adjacent grid
  karts instead of sending both toward the same wider corridor. Availability still wins when only
  one side is feasible. A side rejected by the pre-clearance ordering guard is unlatched for the
  next eligible two-sided assessment.
- The 23:50 dev3 run confirmed that entry speed release and the fixed goal worked, but P1/P2
  still returned to Follow when the rotating first-hairpin frame reported `pass-side gap
  unavailable` after the two-second transient hold. A validated breakout now keeps ownership of
  its same target, side, and explicit line across that re-evaluation. Target replacement,
  position jumps, and explicit forbidden waypoints still cancel continuity.
- The 04:20 dev3 run confirmed that the continuity fix worked: P1 reached rear-clear and Return,
  while P2 stayed in Pass until P1 was classified as a new front vehicle. P1's projected course
  lateral was `-1.64 m`, but the old overlap check treated that course-center coordinate as an
  ego-relative offset and triggered SafetyBrake at `0.26 m` progress distance. Front overlap now
  subtracts ego `e_y` from projected vehicle lateral, with local-relative fallback. The same run
  also showed breakout speed release dropping immediately after the locked target became
  side-by-side; speed ownership now remains latched through ShiftOut/Pass until rear-clear.
- Comparing `output/20260722-051247` with `053122` and `055247` isolated a ShiftOut regression.
  Before live obstacle bounds were hard-applied, P2's first target was `-0.90 m` from current
  `-0.89 m` and the line moved smoothly. Afterwards the first target jumped to about `-1.65 m`
  with `first_epsi=-0.65 rad`; OSQP failed in the same cycle and Recovery returned P2 behind P3.
  ShiftOut now uses the live planner for corridor validity without imposing that unreachable
  first-step interval as a hard state bound.
- `output/20260722-060512/d2` confirmed the ShiftOut correction: P2 reached Pass and latched
  1.15 m lateral separation from P3 without an initial solver failure. Re-enabling the same
  discontinuous vehicle bounds after that latch then moved `first_target` from `-1.20 m` to
  `-1.63 m`, caused eight OSQP failures, and returned P2 behind P3. Explicit-line ownership now
  suppresses those hard bounds for both ShiftOut and Pass; live feasibility and target-ordering
  cancellation remain active.
- `output/20260722-061232/d2` confirmed that the hard-bound correction removed the OSQP failure:
  P2 shifted smoothly from `e_y=-0.89 m` to the fixed right-side target, entered Pass at WP52,
  and latched 1.15 m lateral separation. At WP63 a single live-planner infeasible result still
  caused immediate `Pass -> Recovery`, followed 25 ms later by SafetyBrake and the observed pull
  behind P3. The live planner now uses its own last-valid timestamp and applies the same bounded
  two-second transient hold before declaring the execution corridor blocked.
- `output/20260722-062258/d2` proved that extending the hold alone is insufficient. P2 entered
  Pass at WP51, latched 1.15 m separation from D3 at WP54, and remained `front=0`, `danger=0`,
  `risk=Clear`, with behavior explicitly reporting `committed active pass continuity`. The later
  live planner nevertheless remained infeasible from WP63 through WP68 and forced Recovery exactly
  when the two-second hold expired. A laterally committed Pass now treats that late live-planner
  result as diagnostic-only; the next run separately exposed the target-ordering guard.
- `output/20260722-063146/d2` confirmed that the committed-Pass live-corridor correction worked:
  P2 entered the right Pass at WP52, latched 1.15 m separation from D3 at WP55, ignored the live
  corridor dropout at WP64, and remained in Pass through WP72. At WP73 the target-side ordering
  check changed behavior from Overtake to Cruise (`no relevant vehicle`) and forced
  `Pass -> Recovery`; Follow then pulled P2 behind D3. The ordering check now remains authoritative
  only until lateral separation is latched.
- `output/20260722-064048/d2` confirmed only controller-internal pass completion. D2 entered the
  right Pass at WP52, latched 1.16 m lateral clearance at WP57, changed the projected common-course
  ordering from `+0.16 m` at WP105 to `-0.84 m` at WP108, and completed Return at WP131. Video
  review showed that P2 did not physically pass P3, so projected ordering and FSM completion were
  a false-positive acceptance signal.
- The false positive exposed inconsistent lateral geometry: the old 0.75 m target separation,
  1.2 m base line offset, and 1.15 m lateral-clear latch were smaller than the 1.55 m V2X inflated
  center separation before covariance. The 1.45 m physical combined kart width is retained;
  prediction margin is reduced to 0.05 m, wall clearance to 0.72 m, line target separation is
  raised to 1.40 m, and lateral-clear latch to 1.35 m for the next aggressive dev3 trial.
- The same run isolated the remaining issue to D1. D2 was already to D1's right (`ego e_y` about
  `-0.32 m`, target lateral about `-0.46 m`), but both corridors were feasible and the wider-side
  rule selected right. The ordering guard rejected it almost immediately, followed by repeated
  right-side `ShiftOut -> Recovery -> Follow` cycles. The stagger-aware preference selects left in
  that geometry and the rejection path now clears the failed side latch.
- The prior explicit line ignored the gap planner's midpoint and instead recomputed
  `target_lateral + minimum_separation`, which biased the ego path toward the front kart even when
  the vehicle-to-wall slot was open. The selected side assessment now transports the first valid
  adjusted interval midpoint into OvertakeLine and latches it at ShiftOut entry.

## Verification

- `make autoware-build`: PASS, 25 packages built.
- Final Docker CTest after the common-course lateral and breakout speed-ownership fixes: PASS,
  22/22 CTest entries, including the ego-relative lateral regression and the
  target-change/position-jump/forbidden-WP cancellation regression.
- Docker build after the dynamic corridor and heading-reference correction: PASS, 25 packages.
- Docker package tests after the correction: PASS, 22/22 CTest entries. The final targeted
  `test-result` base reported 537 tests, 0 errors, 0 failures. An earlier workspace-wide aggregate
  also passed but printed an unrelated stale `build/joycon_contract_guard/package.xml` lookup
  warning, so the final check was scoped to this package's result directory.
- Docker build after the final ShiftOut/Pass hard-bound correction: PASS, 25 packages.
- Docker package tests after the final correction: PASS, 22/22 CTest entries and 560 tests,
  0 errors, 0 failures, 0 skipped.
- Docker build after adding the bounded live-corridor hold: PASS, 25 packages.
- Docker package tests after the bounded live-corridor hold: PASS, 22/22 CTest entries and
  560 tests, 0 errors, 0 failures, 0 skipped.
- Docker build after making committed-Pass live-corridor loss diagnostic-only: PASS, 25 packages.
- Docker package tests after the committed-Pass correction: PASS, 22/22 CTest entries and
  561 tests, 0 errors, 0 failures, 0 skipped.
- Docker build after limiting target-ordering cancellation to pre-latch: PASS, 25 packages.
- Docker package tests after the target-ordering correction: PASS, 22/22 CTest entries. The
  package-scoped `test-result` base reported 539 tests, 0 errors, 0 failures, 0 skipped.
- Docker build after adding stagger-aware start-grid side arbitration: PASS, 25 packages.
- Docker package tests after the side-arbitration correction: PASS, 22/22 CTest entries and
  540 tests, 0 errors, 0 failures, 0 skipped.
- Targeted Docker `clang-format --dry-run --Werror` for the newly added helper, declaration, and
  regression-test ranges: PASS. The whole files contain older unrelated formatting differences,
  so the check was intentionally scoped to this correction.
- Aggressive physical-separation parameter alignment: YAML parse PASS; `make autoware-build`
  PASS with 25 packages. Runtime/video acceptance remains pending the next dev3 run.
- Corridor-center line ownership: YAML parse PASS; `make autoware-build` PASS with 25 packages.
  Docker package tests PASS with 22/22 CTest entries and 563 tests, 0 errors, 0 failures,
  0 skipped. Runtime/video acceptance remains pending the next dev3 run.
- `git diff --check`: PASS.
- Host `clang-format --dry-run`: not run because `clang-format` is not installed on the host. The
  ROS build and package tests compiled all changed C++ successfully.

## Next dev3 acceptance

For P1/P2 at the first `start-grid breakout` transition:

- Do not accept `locked_s < 0`, Return, or line completion alone as a successful pass. Confirm in
  video that P2's whole body reaches the selected side and advances beyond P3 without contact or
  being pulled back behind it.

- With the reproduced grid geometry, P1 should select left (`side=1`) because P2 is to its right,
  while P2 should select right (`side=-1`) because P3 is to its left. A one-sided feasible gap may
  override that preference.
- The repeated D1 loop of `locked target entered selected pass-side line` followed by
  `ShiftOut -> Recovery` on the same right side must disappear. If an entry side is rejected before
  lateral clearance, one `re-evaluating both sides` log is acceptable, but the next eligible entry
  must not blindly reuse it.

- V2X debug should show `desired_v` equal to the active domain start maximum, `speed_cap=0`, and
  `cap_release=1` immediately rather than after `ShiftOut -> Pass`.
- OvertakeLine debug should show a finite `corridor_goal` that remains constant while
  `target_last_lateral` changes through the first hairpin.
- After the initial corridor is validated, debug should report `latched start-grid breakout
  continuity` instead of `active pass transient gap hold` and must not transition to Follow when
  that two-second hold would previously expire. Target replacement, position jump, explicit
  forbidden WP, or normal line completion may still end the maneuver.
- After `front-overlap latch` makes the locked target side-by-side, `grid_breakout=1`,
  `speed_cap=0`, and `cap_release=1` should remain active through rear-clear/Return instead of
  dropping back to the front+closing cap.
- A vehicle on another lateral line must not become a hard-center front hazard solely because its
  path-center coordinate is near zero. Debug should use projected vehicle lateral minus ego
  `e_y`; the 04:20 P2/P1 example is approximately `-1.64 - 1.38 = -3.02 m` and therefore must not
  trigger the previous P2 SafetyBrake at WP75.
- During ShiftOut and an unlatched Pass, a transient runtime-planner dropout should log
  `live execution corridor loss held`; persistent loss beyond two seconds should still enter
  Recovery. After `Pass front-overlap exclusion latched`, the same dropout should instead log
  `live execution corridor is diagnostic-only for committed Pass: active=1` and must not cause
  `Pass -> Recovery` by itself.
- Before lateral separation is latched, the previous D1/D2 geometry (`side=-1`, target-relative
  lateral about `-0.35 m`) must log `locked target entered selected pass-side line`; the successful
  D2/D3 ordering (about `+0.67 m`) must not trigger it. After
  `Pass front-overlap exclusion latched`, a rotating ordering change must not cause
  `Overtake -> Cruise` or `Pass -> Recovery`; the line should continue to rear-clear/Return unless
  another explicit hard guard fires.
- OvertakeLine debug now includes finite `first_epsi`; it should have the sign of ShiftOut/Return
  lateral motion and approach zero on a constant-offset Pass.
