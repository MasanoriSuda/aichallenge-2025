# Design

## Causal chain

`SteeringReport` quantization / asynchronous receipt
→ noisy finite-difference steering rate
→ constant-rate projection over observation age plus control delay
→ false physical steering origin
→ retained reachability rejects the certified plan as `steering-unreachable`
→ canonical Emergency command
→ command/measurement reversal on the next cycle
→ repeated normal/Emergency authority loss.

The solver and physical wall proof remained successful. They were downstream
consumers of a bad state origin, not the root cause.

## Producer correction

Let:

- `delta_m` be the latest measured tire angle;
- `delta_c` be the last successfully published tire-angle command;
- `T` be measurement age plus the existing state-prediction delay;
- `r_max` be the existing physical steering-rate limit.

The control-origin state is:

```text
reachable_step = r_max * T
delta_origin = delta_m + clamp(delta_c - delta_m,
                               -reachable_step,
                                reachable_step)
```

This is a zero-order-held committed input passed through the existing bounded
actuator model. It never identifies `delta_c` as the measured state, and it
never assumes a noisy derivative persists for the whole delay.

## Deletion

- Remove measured finite-difference rate from `Request` and `PhysicalState`.
- Remove the controller member and callback calculation that existed only for
  that projection.
- Replace `rate_clamped` telemetry with committed-input reachability telemetry.

## New branch/configuration count

- Production branches: 0
- Configuration parameters: 0
- Normal authorities: 0

## Acceptance

- Unit tests cover reachability-limited motion, target reached, opposite-sign
  motion, stale input, and invalid input.
- Full package tests and `make autoware-build` pass.
- In `make dev2`, `physical_origin` stays between measured steering and the
  last published command, retained `steering-unreachable` no longer chatters,
  and AWSIM reaches `start` without legacy authority.

## Rollback

- Baseline commit: `937bbeb`
