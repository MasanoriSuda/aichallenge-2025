# Requirements

## Purpose

Move expensive MPCC-lite tactical side/corridor/Mission generation out of the
40 Hz control callback. The asynchronous latest-only worker becomes the sole
owner of tactical candidate generation while the live callback consumes only a
fresh, matching result.

## Constraints

- Keep target observation, current hard wall/vehicle/emergency guards and
  low-level MPC execution in the live callback.
- Keep the current frozen/last-feasible Mission while a new worker result is
  pending; never wait for the worker.
- Reject worker results whose target, phase, side, generation, epoch or age no
  longer matches the live context.
- Preserve synchronous planning when the asynchronous worker is disabled.
- Do not change ROS topics, messages, launch entry points or evaluation output
  contracts.

## Acceptance criteria

- With `v2x_overtake_mpcc_lite_async_worker_enabled: true`, ordinary live-side
  assessment does not run DP/corridor/Mission candidate generation.
- Worker-side evaluation still runs the complete assessment.
- A fresh accepted result carries complete left/right tactical assessments,
  not only the selected Mission.
- Existing tests and package build pass.
- Runtime logs identify the tactical owner as the worker.
