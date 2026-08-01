# Requirements

## Purpose

Prevent a kart that is still heavily misaligned with the reference path from timing out of the
solver-independent low-speed rejoin and becoming permanently stopped behind another recovery kart.

## Evidence

In `output/20260801-110947/d2/autoware.log`:

- P2 entered `LOW_SPEED_REJOIN` after 0.201 m with `e_psi=-1.601 rad`;
- the bounded rejoin improved heading continuously to `e_psi=-1.154 rad`, but the absolute
  five-second timeout fired before alignment completed;
- the following Reverse reassessment was V2X-blocked by P1 and ended in
  `clearance_wait_timed_out`;
- the normal MPC remained infeasible, reaching 2280 consecutive fallback failures.

P1 then also stopped because P2 occupied its checked Forward/Reverse recovery corridors.

## Constraints

- Simulation-race aggressive recovery only bypasses normal-MPC solver health.
- Invalid input, stale odometry, disabled control, hard stop, footprint, swept-path, V2X, boost,
  gear, speed, and distance gates remain authoritative.
- A rejoin may extend only while normalized path-alignment error makes material progress.
- A stalled rejoin must still time out and return to the existing bounded reassessment.
- Preserve ROS topics, services, message types, launch contracts, and evaluation outputs.

## Definition of Done

- `LowSpeedRejoin` uses `timeout_sec` as a no-material-progress timeout, not an unconditional
  cutoff while heading/lateral alignment is improving.
- A persistent solver fallback cannot prevent an otherwise recoverable aggressive `SafeStop`
  from reaching its existing checked retry path.
- Non-aggressive and non-recoverable fail-safe behavior remains unchanged.
- Unit tests, package build, and formatting checks pass.
