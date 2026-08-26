# Requirements

## Objective

Explain and correct the clean Track/Cruise wall-contact regression observed in
`output/20260826-182334` without tuning wall margins, solver settings, or
controller weights.

## Observed failure

- No V2X vehicle constrained the ego vehicle.
- At decision 1641, while travelling at 8.21 m/s, the exact 0.13 s
  measured-to-control prefix became wall-blocked.
- Canonical Cruise authority correctly closed and Emergency braking was
  published, but the committed prefix was already unavoidable.
- The current pose then entered hard-wall contact, the canonical QP reached its
  iteration limit, and stuck recovery took authority.
- Immediately before the event, desired steering was about 0.31--0.34 rad,
  measured steering was about 0.20--0.23 rad, and the semantic steering origin
  assumed about 0.29--0.31 rad.
- The first seven-state dynamic Gates still failed:
  `output/20260826-202338` reached wall contact at about 58.5 s, and
  `output/20260826-203435` diverged at about 22--25 s.
- In the second run, one accepted QP trajectory began near -0.097 rad while
  the command extracted from the same steering-rate sequence began near
  -0.365 rad.  The wall proof therefore certified a trajectory different from
  the one sent to AWSIM.

## Hypotheses

### H1: optimistic actuator-state projection

The semantic steering origin moves measured steering toward the
committed desired command at the configured steering-rate limit.  The actual
AWSIM steering response is slower, so the solver and wall certificate assume
more curvature than the vehicle produces.

Evidence required:

- bag-derived desired and measured steering over the first wall event;
- measured steering response versus the projected reachable step;
- first wall failure must occur on the understeer/outward side predicted by
  the mismatch.

Refutation: measured steering reaches the semantic origin within the sealed
prediction interval and the physical trajectory still diverges.

### H2: wall-map false positive

The delay-prefix wall collision is caused by a wall-grid or pose-frame error.

Refutation: the vehicle subsequently remains in current-pose hard contact and
AWSIM/recovery observations agree with the same wall side.

### H3: solver or callback overload

The vehicle leaves the track because commands are delayed by computation.

Refutation: callback runtime remains below the 25 ms period before the first
wall event and the QP maximum-iteration cascade starts after contact.

## Constraints

- Do not change wall clearance, steering-rate limits, OSQP settings, horizon,
  weights, or speed parameters in this Slice.
- Do not add a normal fallback or a second authority.
- Keep desired-command publication continuity distinct from physical steering
  state.
- A behavior change is permitted only after the bag evidence identifies the
  incorrect producer contract.

## Definition of Done

- Root cause is supported by bag and code evidence and competing hypotheses
  are explicitly addressed.
- The physical and yaw-response steering states at the prediction origin are produced
  by a contract whose assumptions are observable in AWSIM.
- Desired publication continuity remains exact and independently certified.
- The certified command-steering sequence originates only at the last
  successfully serialized physical-equivalent command and is the sole source
  of subsequent command publication.
- Measured/yaw-derived steering initializes only the response state and cannot
  re-base the command-rate integral.
- Focused unit tests, full package tests, and `make autoware-build` pass.
- A clean `make dev` run completes the Track/Cruise Gate without
  delay-prefix wall contact, current-pose wall contact, or Recovery.
