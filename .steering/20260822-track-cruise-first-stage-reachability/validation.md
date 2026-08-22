# Validation

## Baseline evidence

Run: `output/20260822-181304`

Decision 3767 rejected the Track/Cruise shadow candidate at swept path index 1, stage 0, wp260.
The current and candidate poses were individually clear, while their interpolated transition was not.
The candidate diagnostic was:

```text
distance=0.990 m
lateral=-1.701 m
bounds=[-2.460, 4.519] m
reserve=0.759 m
heading_offset=0.136 rad
progress=257.016/258.006 m
pose=(89649.406, 43159.745, -0.417)
```

At decision 3769 the production pose was `(89648.444, 43160.391, -0.649)` and had two hard-wall
contact cells. These records do not yet contain enough first-input and state-zero provenance to choose
between H1--H3.

## Failure-first and contract tests

- The first-stage integration tests failed to compile before the typed integration contract existed.
- The complete Frenet-pose tests failed to compile before projection/reconstruction existed.
- `test_mpcc_execution_contract`: 22/22 passed after implementation.
- `test_mpcc_progress`: 55/55 passed and preserves signed solved lag at every stage.

## Diagnostic run before complete-lag correction

Run: `output/20260822-192640`

At decision 3808, the delay-compensated control pose projected exactly into the state-zero course
frame:

```text
state0_error=0.656 m / 0.000 rad
projected_lag=-0.656 m
projected_state0_error=0.000 m / 0.000 rad
solved_lag=0.562 m
theta-only endpoint error=0.095 m
lag-added endpoint error=0.489 m
```

This proved that the raw-vs-state difference was expected prediction delay, while the five-state x0
still discarded a real along-track residual. It also proved that adding solved lag only at the wall
certificate would be wrong: x0=0 and lag omission were cancelling one another.

## Paired correction

- Added invertible `PlanarPose <-> FrenetPose` projection/reconstruction.
- Track/Cruise shadow x0 now carries the projected initial lag.
- `ExtendedExecutionTrajectory` and the aligned shadow trajectory retain solved lag.
- The physical wall certificate reconstructs lag-aware poses when complete five-state provenance is
  present. Legacy/production trajectories without lag preserve their existing behavior.
- The diagnostic separately validates raw-to-predicted and predicted-to-first-stage paths.

## Dynamic validation after correction

Run: `output/20260822-194818`

- shadow eligible/solved: 3,684/3,684;
- physically certified: 3,659 (99.32%);
- rejects: candidate hard contact 2, swept 1, current-pose contact 22;
- first-stage classified events: 3;
- callback overruns: 1;
- authority remained `shadow`, `selected=0`.

All three first-stage events had the same classification:

```text
prediction_wall=valid/blocked/collision
control_wall=valid/blocked/collision/0/1
```

At decision 3783, lag-aware reconstruction reduced the endpoint position residual from 1.101 m to
0.148 m. The raw-to-predicted prefix nevertheless collided, and the predicted-to-control path was
already blocked at its first pose. Decisions 3784 and 3785 repeated the same result.

## Root-cause conclusion

- H3: confirmed and corrected as an incomplete five-state coordinate contract.
- H2: refuted for the measured failures; the control-derived rollout also collided.
- H1: confirmed, but its upstream owner is the legacy production trajectory before handoff. By the
  rejection cycle, no new first input can recover the 0.13 s predicted pose.

No wall margin, speed, steering rate or solver weight was tuned. No rejected candidate was made
selectable. Slice 3 must prevent late handoff by starting/retaining canonical authority while a fresh
certificate still exists.

## Build and regression

- `make autoware-build`: 25 packages built successfully.
- Correct workspace test tree: `colcon test --packages-select multi_purpose_mpc_ros` passed 33/33
  CTest targets.
- `colcon test-result --verbose`: 1,565 tests, zero errors, failures or skips.
