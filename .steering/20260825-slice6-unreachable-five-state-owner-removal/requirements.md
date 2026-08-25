# Requirements

## Purpose

Physically delete the unreachable five-state Overtake normal publisher and
retained/async selector, and stop labelling explicit Emergency output as a
five-state solve. Keep the live tactical pre-entry Gate A unchanged until a
six-state replacement can be promoted atomically.

## Root evidence

- `get_control()` dispatches every supported normal intent only through
  `rate_resolved_normal_production_control()`.
- `canonical_normal_control()` has one definition and zero call sites.
- `evaluate_overtake_async_shadow()` has one definition and zero call sites;
  its submit, retained evaluator, mailbox, selector telemetry and plan-store
  publication are reachable only from that dead root.
- `canonical_normal_emergency_stop()` records
  `VelocityProgress5State` despite performing no MPCC solve and publishing an
  explicit Emergency override.
- The live left/right tactical worker still solves a five-state pre-entry
  artifact. It is consumed only as Mission identity/physical evidence; its
  actuation is prohibited from gating the six-state publisher.

## Required behavior

- No reconnectable five-state normal publisher remains.
- No dead five-state retained/async selection lifecycle remains.
- Emergency trace identity is `Unresolved`, never a fabricated solved
  formulation.
- The live five-state tactical Gate A remains fail-closed and commandless.
- Supported normal intents continue to publish only through the shared
  six-state producer; Emergency and Recovery stay external overrides.

## Non-goals

- Do not remove or bypass the live pre-entry Gate A in this Slice.
- Do not promote a Mission without dynamic evidence.
- Do not tune solver, horizon, weight, clearance, timeout or speed parameters.
- Do not change ROS interfaces or Stuck/AWSIM Recovery.

## Gate

- Failure-first source contracts identify all dead roots and the false
  Emergency formulation.
- Exact-symbol search shows no retired publisher/async selector surface.
- Existing tactical Gate A source contract still proves that it cannot own
  actuation.
- Build and full package tests pass.
- A bounded run shows six-state normal publication or explicit unresolved
  Emergency, never five-state normal publication.
