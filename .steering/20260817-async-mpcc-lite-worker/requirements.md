# Requirements

## Goal

Keep the 40 Hz tracking/control callback independent of the expensive
MPCC-lite tactical rollout. The latest run showed D1 callback overruns in
18.7% of active cycles while tactical evaluation reached 50--80 ms.

## Required behavior

- Evaluate the expensive MPCC-lite left/right tactical refresh on a dedicated
  latest-only worker.
- Never wait for the worker in the control callback.
- Keep at most one pending snapshot; a newer snapshot replaces an older
  pending one.
- Accept a result only when target, Mission generation, phase, side and age
  still match the live controller context.
- Continue the current Mission or legacy live assessment while no fresh worker
  result exists.
- Revalidate all promoted Mission/prefix results through the existing runtime
  hard guards and atomic promotion code.
- Stop and join the worker during controller destruction.

## Constraints

- Do not change ROS topics, message types, launch entry points or evaluation
  contracts.
- Do not move the 40 Hz tracking MPC/OSQP solve off the control callback.
- Do not weaken wall, target-footprint, no-return or emergency guards.
- Preserve the user's generated `aichallenge/result-summary.json` change.
