# Requirements: SafetyBrake lateral authority

## Objective

Prevent an explicit SafetyBrake Stop from carrying a stale constant steering
command through the full braking distance. Stop remains the sole emergency
wire authority, but its lateral command must converge toward the current base
path while the vehicle is still moving.

## Failure-first evidence

In `output/20260827-214537/d1/autoware.log`:

- decision 3048 entered Stop at 5.15 m/s with no wall contact and a 0.40 m
  required wall clearance;
- Stop repeatedly published the same physical steering, `-0.159 rad`;
- the reference curvature changed from approximately `-0.098` to `-0.032`
  rad/m while the command did not change;
- wall state degraded from `clear` to `near/0.31 m`, then `front/0.00 m`;
- at decision 3106, while still moving at 0.84 m/s, the footprint made wall
  contact;
- only afterwards did the paused ShiftOut become persistently impossible to
  reconnect.

## Invariant

An explicit Stop may own longitudinal emergency braking without owning an
unbounded zero-order-hold lateral command. While moving, it must rate-limit
toward the current reference-path steering target; if that target cannot be
constructed it must rate-limit toward neutral. Holding the last command is
permitted only once longitudinal motion has ceased.

## Constraints

- Do not weaken SafetyBrake distance, wall clearance, steering rate, solver,
  or current-world proof.
- Do not promote a latent normal shadow to production during Stop.
- Do not mark a normal artifact executed when Stop changed its longitudinal
  command.
- Do not add a timeout, lease, grace period, parameter, or legacy controller.
- Preserve Recovery as a later final-publication override.
- Keep generated result files uncommitted.

## Definition of Done

- A pure contract test rejects moving constant-steering Stop ownership.
- Explicit Stop uses the established path-feedback target with the existing
  steering-rate and lateral-acceleration limits.
- At-rest Stop remains stable and normal-intent Emergency behavior is not
  broadened.
- Focused tests, full package tests, and `make autoware-build` pass.
- A bounded `make dev2` run shows Stop steering changing with the path and no
  repetition of the decision-3048-to-3106 wall-contact chain.
