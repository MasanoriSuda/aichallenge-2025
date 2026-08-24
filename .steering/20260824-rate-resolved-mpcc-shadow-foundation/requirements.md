# Requirements

## Objective

Define and verify the rate-resolved actuation model required to remove the
canonical five-state solver/publisher time-base mismatch. This Slice is a pure
shadow foundation: it must not change production authority, commands, config,
or the existing five-state QP.

## Root-cause contract

The current five-state formulation uses curvature as a piecewise-constant
input over a coarse prediction stage, while canonical execution publishes that
stage value as an instantaneous 40 Hz steering command. A stage endpoint is
therefore being used as an actuator sample. Restricting every coarse stage to
one 40 Hz steering step was dynamically falsified in
`output/20260824-232452` because it made ordinary Track/Cruise infeasible.

The replacement model must represent steering evolution explicitly:

- state: `[e_y, e_lag, e_psi, v, theta, delta]`;
- input: `[a, delta_dot, v_theta]`;
- curvature in vehicle dynamics: `tan(delta) / wheelbase`;
- steering command at publication time: the certified steering state reached
  by the bounded steering-rate input, never a post-solve clamp.

## Constraints

- Preserve all ROS interfaces and production authority.
- Do not add a feature flag, fallback, timeout, lease, or parameter tuning.
- Do not reinterpret or mutate the existing five-state primal.
- Do not modify or commit `aichallenge/result-summary.json`.
- Keep the new mathematics isolated and independently testable before QP
  assembly, warm start, canonical artifact, or publisher migration.

## Definition of Done

- A pure six-state temporal Frenet linearization exists with explicit units and
  indices.
- Steering angle is a state; steering rate is the only lateral actuator input.
- A pure stage sampler proves that any 40 Hz sample is derived from the same
  constant-rate stage contract.
- Tests cover the affine reference point, steering-state integration, heading
  coupling, rate/bound rejection, and intermediate publication samples.
- `git diff --check`, package build, and the full package tests pass.
- No production executable links or calls the new module in this Slice.
