# Requirements

## Purpose

Reduce avoidable `ShiftOut` / `Pass` aborts observed in the 20260813-170508 run without changing the low-priority reverse recovery behavior.

## Scope

- Repair a receding-horizon trajectory after static-wall and lateral-acceleration post-validation changes it.
- Keep the fastest physically feasible repaired trajectory instead of immediately entering `Recovery`.
- Stop enforcing target-separation bounds once current and predicted kart footprints are already separated during `Pass`.
- Keep physical wall infeasibility, emergency risk, invalid target continuity, and blocked corridors as hard failures.
- Add bounded debug state for the repair path and any temporary velocity cap.

## Constraints

- Do not change ROS 2 topics, messages, services, launch entry points, or result schemas.
- Do not change reverse/stuck recovery behavior.
- Do not execute a trajectory that fails footprint/static-wall or lateral-acceleration validation.

