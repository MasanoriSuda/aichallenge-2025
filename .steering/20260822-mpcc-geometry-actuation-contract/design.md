# Slice 2b design

## Causal statement

The same transition currently has two distances: raw `stage_geometry` supplies identity and physical
certification, while `resolve_stage_distances()` supplies a positive effective distance to temporal
dynamics. At the circular seam this makes the solver and certifier disagree. Separately, the
five-state result is flattened into a legacy two-input layout, so optimized acceleration loses its
type and stage-1 velocity is mistaken for the legacy target-speed input. These are data-contract
defects upstream of parameter tuning.

## Effective progress geometry

Create a pure resolver:

```text
raw waypoint transition identity + effective temporal stage distances
                              |
                              v
effective StageGeometryIdentity[] + rebuilt cumulative distance + fingerprint
```

Waypoint `from/state` identities remain unchanged. Only the transition/cumulative distances are
replaced with the already-resolved values that the five-state dynamics actually consume. Invalid
sizes, non-finite/non-positive effective distances or broken waypoint chains are rejected.

`MpcProblem` keeps legacy raw geometry and a distinct effective progress geometry. Consumers select
by formulation:

- legacy 3-state: raw geometry;
- progress 3-state / velocity-progress 5-state: effective progress geometry;
- five-state physical certificate: effective cumulative distances.

This is not a tolerance bypass. Real wall/contact checks still evaluate every stage and swept pose.

## Typed actuation proposal

The first executable sample of a five-state solution is represented as:

```text
decision/problem/solution identity
predicted_speed_mps       = x[1].v
acceleration_mps2         = u[0].a
curvature_radpm           = u[0].kappa
steering_tire_angle_rad   = atan(wheelbase * curvature)
virtual_progress_speed    = u[0].v_progress
```

This proposal is diagnostic in Slice 2b. It is joined after all existing final command arbitration
using the same decision ID. Telemetry distinguishes:

- predicted-speed versus final target-speed delta;
- optimized-acceleration versus final acceleration delta;
- optimized-steering versus final published steering delta.

No field is published from the proposal before a later authority-promotion slice explicitly defines
the final post-processing contract.

## Files

- `mpcc_execution_contract.hpp/.cpp`: effective progress geometry resolver.
- `mpcc_progress.hpp/.cpp`: typed actuation proposal extractor.
- corresponding focused tests first.
- `mpc_controller_cpp.cpp`: store formulation-specific geometry, use it in shadow certificate and
  expose/join proposal telemetry without authority.
- update `docs/spec/mpc-integration.md` after validation.

## Deletion gate

`convert_extended_solution_to_legacy()` remains only for existing prediction-layout consumers in
this slice. Slice 5 removes it from live execution. Any command-difference field whose name implies
equivalent speed semantics is removed or renamed now.
