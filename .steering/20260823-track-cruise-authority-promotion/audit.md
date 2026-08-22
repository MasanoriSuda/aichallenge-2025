# Static authority audit

## Current producer

- `MPC::get_control()` builds the ordinary problem and records an initial formulation.
- The existing live path may run five-state extended MPCC, convert it to a legacy vector, or fall
  back to three-state/legacy MPC.
- Track/Cruise canonical evaluation is called only after one of those production solutions exists.
- The canonical evaluator itself does not require the production vector except for comparison
  telemetry; this dependency can be removed from the authority path.

## Current command transport

- `get_control()` returns `std::pair<Eigen::Vector2d, double>`.
- `Eigen::Vector2d` contains target speed and steering only.
- Canonical optimized acceleration, virtual-progress speed, plan ID and selector source do not cross
  this boundary.
- The node callback recomputes/filters acceleration and steering before publication.

## Current retained path

- Retained current-world proof, candidate selection and actuation extraction are dynamically
  demonstrated in `output/20260823-014243/`.
- `TrackCruiseShadowCycleResult` retains only status/identity flags after extraction; it does not
  return the extracted retained actuation.
- Therefore retained production fallback cannot be implemented by choosing the existing pending
  fresh telemetry object.

## Current reset/failure behavior

- `safe_failure_control()` and final callback logic can enter solver crawl or bounded continuation.
- `publish_failsafe_command()` resets control history and clears the canonical plan store.
- These behaviors are compatible with the old solver contract but cannot sit between a failed
  fresh canonical candidate and the already proven retained canonical selector.

## Interface impact

- No ROS topic/service/message or launch contract needs to change.
- Internal C++ control-result and final-source tracing contracts must change.
- Recovery remains an override and `/control/command/control_cmd` remains the sole output.

## Audit conclusion

Gate A and Gate B prove the canonical producer, plan lifecycle, current-world proof and selector.
The remaining blocker is internal command transport and late post-processing.  Authority promotion
is now a bounded architectural change, but it is material: it removes Track/Cruise legacy normal
authority and changes the commands reaching the vehicle.  Explicit approval is required before
implementation.

## Approved implementation gate (2026-08-23)

- Approval: the user explicitly approved production authority promotion after reviewing this
  steering design.
- Baseline commit: `a845eb5` (`docs(mpcc): define Track Cruise authority promotion`).
- Gate A/Gate B evidence: `output/20260823-014243`, including fresh rejection followed by
  current-world-certified retained selection at decision `20804`.
- Root producer: the velocity-progress five-state problem, normalized primal, physical certificate,
  immutable canonical execution plan and current-world retained proof.
- Mask/bypass being removed: Track/Cruise live conversion to the legacy 3-state layout, cycle-local
  3-state/legacy solve fallback, extended-mode handoff smoothing, and solver crawl/continuation as
  normal authority after canonical rejection.
- Files expected to change: the canonical execution contract and tests, `mpc_controller_cpp.cpp`,
  this steering directory, and the integration specification. No launch, ROS topic/message,
  parameter or evaluator-system file is in scope.
- New runtime flags/leases/retries/tuning parameters: zero.
- Rollback boundary: revert the authority-promotion implementation commit; do not retain both
  canonical and legacy Track/Cruise authority behind a flag.
- User artifact exclusion: `aichallenge/result-summary.json` remains unstaged and untouched.

## Failure-first evidence

Before adding the command contract, `make autoware-build` failed in
`test_mpcc_execution_contract.cpp` because `CanonicalActuation`,
`CanonicalNormalCommandReason`, `build_canonical_normal_command()` and the exact-actuation check did
not exist. This is the intended proof that the old `{speed, steering}` boundary cannot express the
approved authority contract. The failure occurred before any production source was changed.

## Implementation audit

- `MPC::get_control()` now exits through canonical fresh/retained/Emergency resolution before the
  live extended-to-legacy conversion, three-state/legacy solve and handoff smoothing for an eligible
  Track/Cruise cycle.
- A fresh plan replaces the retained atomic store only after cursor, candidate, selector, actuation,
  exact-actuation join, world-prediction reconstruction and command construction all succeed. A
  post-extraction rejection therefore cannot poison retained production authority.
- Retained authority carries its original problem/solution identity and the current decision's
  current-world execution certificate separately.
- The callback consumes optimized acceleration and steering directly. Normal acceleration/steering
  reconstruction, low-pass and steering-rate post-processing are bypassed; configured physical
  limits remain fail-closed checks rather than silent mutation.
- The final execution trace now requires canonical command identity for a five-state certified
  normal source. It records plan ID, current execution-certificate decision ID and fresh/retained
  source. Missing or mismatched retained current-world identity cannot be labelled canonical.
- Canonical rejection disables solver crawl and bounded continuation and produces the explicit
  canonical Emergency path. Stuck Recovery remains a whole-command supervisor override.

## Static verification evidence

- The eligible Track/Cruise early return precedes `convert_extended_solution_to_legacy()`,
  `solve_problem()` and `extended_mode_handoff_.resolve_velocity()` in `get_control()`.
- Those legacy calls remain reachable only for the not-yet-promoted Follow/Overtake and other
  non-eligible intents; they were not duplicated behind a new flag.
- No ROS topic, message, launch, parameter, solver setting, wall margin or control gain changed.
- `make autoware-build` succeeded after production integration.
- The package test run passed all pre-existing suites except one new retained fixture that lacked a
  current-world proof ID; correcting the fixture made the focused formal result
  `1556 tests, 0 errors, 0 failures, 0 skipped`.

## Dynamic production evidence

### Exact canonical publication root cause

The first production run, `output/20260823-062054`, exposed a contract breach below the canonical
selector rather than a path-planning failure.  The bicycle model and physical certificate consumed
the optimized tire angle directly, but the generic publisher multiplied it by
`steering_tire_angle_gain_var=1.5`.  Before correction the trace therefore contained, for example,
`command=-0.36 rad, published_steering=-0.54 rad`; the predicted and executed vehicle could not share
the certified fingerprint.  This is the upstream cause of the previous wp74 loss, not an MPCC
weight or wall-margin defect.

The publication contract now preserves the physical angle for both canonical normal authority and
canonical Track/Cruise Emergency Stop.  Legacy normal paths retain their existing calibration while
Recovery remains an explicit whole-command override.  Failure-first linker tests fixed this
authority matrix before the publisher condition was changed.

### Repeated runs

- `output/20260823-062054`: three completed laps, `44.120 / 41.580 / 41.755 s`; no callback overrun
  was reported and the previous wp74 stop did not recur.  Normal canonical traces show raw command
  angle equal to published angle.
- `output/20260823-064933`: three completed laps, `46.471 / 42.465 / 43.400 s`; normal and canonical
  Emergency traces both preserve the exact angle (for example `-0.36 -> -0.36 rad`).  No canonical
  postprocessor mutation was recorded.  Most one-second windows certified 97.6--100% of eligible
  cycles with final actuation deltas exactly zero.
- The second run contained one 487.602 ms startup/lap-boundary callback excursion and one 28.310 ms
  excursion.  These did not stop the car, but the strict zero-overrun acceptance remains open.

### Rejected experiment

An attempted mathematically equivalent row scaling of the mixed-unit QP was tested in
`output/20260823-063519`.  It reduced OSQP convergence to 0%, with roughly 3,200 iterations and
16--22 ms solve time while stationary.  The experiment was stopped immediately and all row-scaling
changes were removed.  It is not part of the implementation or fallback graph.

### Remaining defect separated from this authority fix

Across the two successful three-lap runs the semantic execution check still rejected occasional
OSQP `solved` results whose curvature, acceleration or predicted velocity exceeded that row's own
tolerance.  The downstream rejection is correct; a large global mixed-unit residual can still let
OSQP terminate before every executable row meets its unit-specific contract.  With V2X health
`NoData`, retained current-world proof correctly rejects `obstacle-observation-unavailable`, so these
cycles become explicit one-cycle Emergency Stops rather than an uncertified normal command.

This numerical formulation issue must be handled as a separate failure-first Slice.  The failed row
scaling experiment proves that a global normalization patch is not an acceptable fix.  No solver
setting, controller gain, wall margin, retry, lease or compatibility fallback was added here.

## Static authority conclusion

Eligible Track/Cruise returns from the canonical fresh/retained/Emergency resolver before the live
extended-to-legacy conversion, three-state solve and handoff smoothing.  The remaining legacy calls
serve non-promoted intents only.  Canonical normal and Emergency publication use one typed steering
authority resolver, and Recovery explicitly wins that resolver.  There is no feature flag retaining
the old Track/Cruise normal authority.

Dynamic production promotion is demonstrated, but a single uninterrupted six-lap zero-overrun
acceptance run and the independent mixed-unit solver defect remain open before this Slice can be
declared fully accepted.

## Final static regression result

After the publication fix, `make autoware-build` succeeded.  The complete
`multi_purpose_mpc_ros` package test run passed `1600 tests, 0 errors, 0 failures, 0 skipped`,
including 49 execution-contract tests and 59 progress/formulation tests.  No generated output or
the user-owned `aichallenge/result-summary.json` is part of the implementation change.
