# Design

## Causal questions

1. Did replay use the same wheel-speed contract as runtime?
2. Does offline `base + spatial residual` reproduce the published command?
3. Is the large correction present before contact, or only after the vehicle is
   already trapped?
4. Do clean peers traverse the same course region with materially different
   clearance and command history?
5. Does the already admitted speed-committed recurrent candidate change the
   frozen first interaction in closed loop?

## Instrumentation

Extend `audit_spatial_candidate_replay.py`; do not add a second parser.  Read
the control topic and report, per immutable time window:

- wheel speed and published acceleration/steering;
- embedded base steering;
- learned residual;
- final node-clamped composition;
- nearest published-command parity.

The `focus` window begins ten seconds before the first sustained post-motion
stop.  The older longest-stop timestamp is retained only as evidence of why
the initial diagnosis was invalid.

## Bounded experiment

The next closed-loop A/B keeps the base, spatial authority, acceleration,
speed cap, world and all longitudinal thresholds unchanged.  It enables only
the existing default-off recurrent correction candidate at its admitted
`+/-0.24 rad` bound.  This candidate was trained from independently executed
speed-committed teacher runs and has already passed single/NPC non-regression.

Interpretation:

- d1/d2 pass the first 48 m interaction: temporal lateral authority is the
  primary missing variable;
- d1/d2 fail in the same way: speed-aware longitudinal or a new observation
  contract is still required;
- a new failure appears before the frozen one: recurrent integration is not a
  valid escape and remains default-off.
