# Requirements

## Objective

Keep one seven-state MPCC as the canonical normal-control authority while
removing the synchronous solver from the 40 Hz control callback.

## Evidence

- `output/20260829-003210` proved that the same-cycle seven-state problem is
  physically solvable.  The previous failure was therefore not a missing
  controller formulation.
- `output/20260829-010335` showed direct solves taking roughly 35--230 ms
  inside a 25 ms callback budget.  Direct synchronous production is not a
  viable runtime architecture.
- `.steering/ano/autoware - 2026-08-21T211659.829.log` shows the upper-ranked
  GMPCC updating its main solution at about 7 Hz with 25--57 ms solve times,
  while a separate child process evaluates tactical alternatives.

## Required architecture

- The background latest-only worker is the only producer of fresh normal
  seven-state MPCC artifacts.
- The 40 Hz callback may publish only a current-world-certified prefix of the
  last actually published artifact, or a newly certified worker artifact.
- Track, Cruise, Follow, ShiftOut, Pass, Return and Rejoin use the same
  seven-state formulation.  Restoring the worker must not restore a legacy MPC
  authority.
- A worker result is not authority merely because it solved.  It must pass the
  existing exact physical-wall and current-world/dynamic-obstacle proof.
- Tactical left/right pre-entry evaluation remains asynchronous and cannot
  publish a command.
- Emergency Stop remains an external supervisor, not a second normal
  controller.

## Forbidden changes

- No lease, grace period, timeout or uncertified command hold.
- No solver tolerance, wall clearance or behavior threshold tuning.
- No new fallback controller.
- No direct solve inside the 40 Hz production callback.

## Exit criteria

- Build and focused contract tests pass.
- A bounded dynamic run shows no synchronous normal solve in the callback.
- The normal worker produces certified candidates and the publisher promotes
  only an exactly serialized command.
- Callback timing remains within budget apart from independently identified
  non-solver work.
