# Requirements

## Goal

Keep the 40 Hz control callback responsive while the asynchronous MPCC-lite
tactical worker evaluates an active overtake scene.

The `output/20260817-010031` trial confirmed that the worker is detached and
healthy, but its 75--86 ms typical compute time at a fixed 10 Hz consumes too
much CPU. Callback overruns were concentrated in worker-active windows.

## Requirements

- Use 5 Hz as the normal asynchronous tactical refresh rate.
- Increase the submission interval when the measured worker compute time would
  exceed a configurable CPU-utilization target.
- Bound the adaptive interval so tactical results remain younger than the
  existing 0.50 s admission lease.
- Keep the 40 Hz tracking controller, hard guards and last-feasible trajectory
  independent of a new worker result.
- Apply the same settings to normal development and cloud configurations.
- Expose the effective interval in the existing compact worker diagnostic.
- Add deterministic unit tests for the interval policy.

## Non-goals

- Do not change MPCC path scoring or physical constraints in this change.
- Do not weaken wall, target-footprint or solver guards.
- Do not change ROS topics, services, message types or evaluation interfaces.
