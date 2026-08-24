# Design

## Data ownership

The solver worker remains numerical and map-independent. Its immutable
artifact gains only the solve-time course-progress origin and the nominal
state-stage distances (`0, d1, ..., dN`). This prevents a later consumer from
reconstructing a different time/progress horizon.

## Pure adapter

A small pure module converts a valid artifact into the existing
`ExactPhysicalExecutionTrajectory` representation:

```text
artifact state[1..N]
  relative progress + source origin
  exact lateral / lag / heading / velocity
  exact lateral boxes
  nominal state-stage distance
        -> exact physical trajectory
```

The adapter checks intent and stage-geometry identity against the current
cycle. It does not know ROS, the map, the controller or authority.

## Current-world physical proof

The controller consumes the immutable artifact on its normal thread. Only when
intent and stage geometry still match does it build course-frame knots from the
current reference path and call the established
`solved_mpcc_execution_path_wall_safe(..., SweptFromCurrentPose)` checker.

This deliberately keeps the wall grid and measured pose out of the async
solver worker. It also means the certificate describes the world at
consumption, not merely the worker snapshot.

## Authority boundary

Physical acceptance is telemetry only. It does not create a plan store,
candidate, normal command or publisher path. Retained current-world admission
is a later bounded Slice.
