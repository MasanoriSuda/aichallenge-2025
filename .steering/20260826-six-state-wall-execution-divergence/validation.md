# Validation

## Root-cause evidence

The canonical analyzer in this steering directory joins the first incident in
`output/20260825-235153` across the final control command, steering status,
kinematic state, yaw rate and controller log.

- `/control/command/control_cmd` carries the steering chosen by canonical
  production; command transport is intact.
- Before contact the desired steering is approximately `0.326--0.358 rad`,
  while measured tire steering is approximately `0.205--0.255 rad`.
- Command-derived kinematic curvature is approximately `0.32 1/m`, while the
  measured yaw-rate/speed curvature is approximately `0.11 1/m` at divergence.
- Actual wall distance reaches zero before the sustained QP rejection cascade.

Source tracing shows that the desired predecessor command was also used as the
physical steering state for every fresh, transition, pre-entry and retained
six-state proof. That shared invalid premise explains why retained
revalidation could not expose the mismatch.

## Structural correction

- Added one observed steering-state contract sourced from
  `/vehicle/status/steering_status`.
- The observation is freshness checked and projected onto the existing
  latency-compensated control origin using a steering rate bounded by the
  existing physical model.
- All six-state problem and retained current-world builders bind the projected
  observed state. The desired predecessor command remains publication state
  only.
- Missing or stale steering observation leaves canonical normal authority
  closed; no legacy normal fallback was added.
- The decision log records measured steering, measured/bounded steering rate,
  clamp status, observation age and prediction-origin steering.

No parameter, wall margin, solver setting, horizon, fallback, lease or
authority switch was added.

## Static validation

- `make autoware-build`: PASS
  - 25 packages completed.
  - Only pre-existing scoped-header and setuptools warnings were emitted.
- Full `multi_purpose_mpc_ros` package test: PASS
  - 51 test targets.
  - 1,890 tests.
  - 0 errors and 0 failures.
- New `test_mpcc_steering_state_contract`: 5 tests passed.
- Updated single-authority source contract: 55 tests passed.
- `colcon test-result --verbose` emitted the pre-existing stale
  `build/joycon_contract_guard/package.xml` lookup warning; the authoritative
  result summary still reports 1,890 tests with zero errors and failures.

## Bounded dynamic startup check

`output/20260826-004254` was produced after the canonical build.

- Updated log vocabulary is present (`physical_steering`,
  `measured_steering`, `physical_origin`).
- Steering status becomes available after spawn and supplies every submitted
  Cruise shadow with a finite physical state.
- The run reached AWSIM `Ready` but did not enter vehicle `Start`; therefore it
  does not satisfy the moving-vehicle acceptance gate.

## Required moving-vehicle gate

Run one bounded `make dev` trial through at least the first hairpin and verify:

1. no `Six-state physical steering unavailable` after motion starts;
2. `causal_submit ... physical_steering` and `Steering debug ...
   measured_steering/physical_origin` remain finite;
3. measured steering and command may differ, but solver `delta0` follows the
   logged physical prediction origin rather than the previous desired command;
4. no actual wall contact follows an accepted physical horizon at the original
   incident location;
5. any remaining contact is classified as a steering-response/model
   calibration defect using `measured_rate`, `bounded_rate`, `rate_clamped` and
   curvature telemetry, not hidden by parameter tuning.
