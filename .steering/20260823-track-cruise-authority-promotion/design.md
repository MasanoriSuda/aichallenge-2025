# Design

## Root cause of the remaining integration gap

The canonical producer/selector is complete, but the current production interface is still a
legacy interface:

1. `MPC::get_control()` solves the current production formulation first and returns only
   `{target_speed, steering}`.
2. `evaluate_track_cruise_shadow()` then independently solves/certifies canonical MPCC and uses the
   production solution only for comparison telemetry.
3. Fresh canonical actuation is stored only as pending shadow telemetry.  Retained actuation is
   extracted and then discarded after recording booleans.
4. The controller callback reconstructs acceleration from target-speed error, low-pass filters it,
   low-pass filters/rate-limits steering again, and finally publishes.

Consequently, wiring the existing pending shadow proposal into `u` would lose plan/source identity,
discard retained actuation, and mutate the command after its physical certificate.  That would be a
late handoff, not canonical authority.

## Selected architecture

### 1. Typed canonical cycle result

Refactor the Track/Cruise evaluator to return one optional selected normal command:

```cpp
struct CanonicalNormalCommand
{
  std::uint64_t decision_id;
  std::uint64_t plan_id;
  CanonicalNormalAuthoritySource source;
  ControlIntent intent;
  double predicted_speed_mps;
  double acceleration_mps2;
  double curvature_radpm;
  double steering_tire_angle_rad;
  double virtual_progress_speed_mps;
};
```

Fresh and retained paths must populate the same type only after the existing canonical selector and
actuation extraction succeed.  No separate retained boolean may imply executable authority.

### 2. One Track/Cruise normal resolver

For a Track/Cruise problem, run the canonical five-state producer first.  Resolve exactly once:

```text
FreshCertified   -> exact fresh canonical command
RetainedCertified-> exact retained canonical command
EmergencyStop    -> no normal command
```

The legacy solve is not run for that Track/Cruise cycle.  Emergency Stop is requested when neither
canonical candidate is available; solver-crawl and three-state fallback do not become normal
authority.

### 3. Preserve command identity to the publisher boundary

Replace the anonymous `std::pair<Eigen::Vector2d, double>` result for the promoted path with a
typed control result that carries source and optimized acceleration.  The final trace and publisher
must consume the same object/decision.  Existing topic shape remains unchanged.

The canonical solver already constrains acceleration, curvature and their physical horizon.  The
normal path therefore must not recompute acceleration from speed error or apply a second steering
low-pass/rate limiter.  Emergency Stop and Recovery remain explicit higher-priority overrides and
may replace the complete normal command.

### 4. Keep state updates coherent

When canonical normal authority is selected:

- update the controller's previous steering from the published canonical steering;
- retain the exact canonical prediction for visualization/wall monitoring;
- do not manufacture a legacy `dec` vector merely to reuse old bookkeeping;
- record the canonical problem/solution contract as the published source;
- clear the canonical plan store on external maneuver/reset exactly as today.

### 5. Delete replaced logic in the same Slice

For Track/Cruise only, delete/bypass structurally rather than retain behind a flag:

- `solve_problem()` legacy normal execution;
- three-state progress fallback;
- extended-to-legacy conversion in the live command path;
- `extended_mode_handoff_` velocity smoothing;
- solver crawl/bounded continuation when its only input is a failed Track/Cruise normal solve;
- `legacy-normal-bypass` final-source classification.

Legacy comparison conversion may remain only in disconnected telemetry until Slice 6, but it must
not gate or feed publication.

## Failure-first tests

1. Fresh canonical command preserves decision/plan/source and all five actuation values.
2. Retained command is selected only with current proof and carries retained plan identity.
3. No candidate produces EmergencyStop and no normal command.
4. A current intent mismatch cannot publish a retained command.
5. A postprocessor cannot silently alter canonical acceleration or steering while retaining the
   canonical source label.
6. Recovery override replaces the whole canonical command and source.
7. Track/Cruise routing never calls a legacy solve callback, including fresh numerical rejection.

## Dynamic rollout after approval

1. `make dev`, explicit empty V2X: fresh and retained selection, no legacy-normal source.
2. Repeated single-car six-lap runs: deadline, wall/contact and completion gates.
3. Only after Track/Cruise passes: proceed to Slice 4 Follow/Hold/Stop integration.

## Rejected alternatives

- Assign `pending_track_cruise_shadow_actuation_` directly to `u`.
- Keep legacy solve warm in the same cycle "just in case".
- Publish canonical steering after the legacy callback filters it.
- Recompute acceleration from canonical predicted speed.
- Add a runtime flag to switch between canonical and legacy Track/Cruise.
- Treat the existing solver-crawl path as the retained canonical fallback.
