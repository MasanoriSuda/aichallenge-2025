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
