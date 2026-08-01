# Requirements

## Purpose

Recover P1 after contact with P2 without repeating Reverse maneuvers until the kart leaves the
course or enters an unbounded aggressive-retry loop.

## Evidence

In `output/20260801-090859/d1/autoware.log`:

- four Recovery episodes reached `rejoin_complete`, while the adaptive Reverse target increased
  from 0.8 m to 4.0 m;
- a later MPC failure latched `solver_reverse_only=1`;
- the MPC fallback cleared, but the episode latch remained and `forward_fallback=0` continued;
- actual lateral error grew to approximately 3.97 m while repeated Reverse maneuvers were selected;
- once Reverse rollouts hit static occupancy, the supervisor repeated 34 aggressive retries with
  `direction=Unknown` although the current footprint, Forward path, and V2X corridor were clear.

## Constraints

- Simulation race mode only for aggressive Forward recovery.
- Reverse remains the first response to a coordinated stopped-front collision.
- Do not bypass swept-footprint, course-progress, V2X, boost, odometry, or gear checks.
- A transient solver failure may establish the episode reason; its later recovery must not prevent
  releasing a stale Reverse-only latch after a fully checked failed Reverse attempt.
- Stop an executing Reverse maneuver when measured course lateral error materially worsens, then
  reassess from Drive and prefer a validated course-directed Forward primitive.
- Preserve ROS interfaces and the user's existing `config.yaml` and generated result changes.

## Definition of Done

- A solver-origin Reverse-only episode can unlock after a checked blocked Reverse attempt even if
  the solver has recovered since episode entry.
- Actual Reverse motion that worsens lateral error outside the rejoin envelope is stopped and
  reassessed instead of continuing toward the 4.0 m target.
- The reassessment can select Forward only through the existing static, course, V2X, boost, and
  gear validation.
- Existing Reverse-first behavior and non-simulation behavior remain unchanged.
- Package tests and build pass.
