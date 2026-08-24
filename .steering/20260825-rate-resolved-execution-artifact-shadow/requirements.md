# Requirements

## Objective

Preserve one solved six-state steering-rate MPCC horizon as an immutable,
observation-only execution artifact before any production authority migration.

## Root cause addressed

The established canonical execution plan stores curvature as a control input.
The rate-resolved formulation instead owns steering angle as a state and
steering rate as the lateral input. Reusing the five-state plan would silently
reconstruct steering with `atan(wheelbase * curvature)` and separate the
published command from the six-state solver certificate.

## Required invariant

One accepted shadow result must retain, without lossy conversion:

- the exact solve identity and prediction time origin;
- all `N + 1` six-state predictions;
- all `N` acceleration, steering-rate and virtual-progress controls;
- each exact stage duration;
- the lateral boxes used by that exact QP;
- the accepted physical-row residual certificate;
- semantic-current-steering based piecewise actuator reachability across the
  complete stored horizon.

The artifact remains immutable and observation-only. It must have no route to
the final publisher or canonical normal authority selector.

## Non-scope

- No authority or fallback change.
- No Track/Cruise tuning, OSQP tuning, cadence change or warm start.
- No claim that QP lateral boxes alone are a production physical wall proof.
- No Follow, Overtake, Rejoin or Recovery migration.

## Preserved user state

`aichallenge/result-summary.json` is a pre-existing user change and must not be
edited, staged or committed.
