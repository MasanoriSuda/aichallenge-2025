# Requirements

## Purpose

Repair the six-state MPCC steering prediction origin after run
`output/20260826-042358` showed that finite-difference steering-rate noise was
being extrapolated through the latency horizon and intermittently invalidating
otherwise certified Track/Cruise plans.

## Repaired invariant

The six-state steering state at the control prediction origin must be derived
from:

1. the latest fresh physical steering measurement; and
2. the steering command that was already published during the latency prefix.

A desired command is an actuator input, not a physical state. A measured
finite-difference rate is diagnostic evidence, not a valid constant-rate input
over the whole prediction delay.

## Scope

- Replace constant measured-rate extrapolation with bounded actuator motion
  toward the last published steering command.
- Use the same physical-origin steering for fresh solve, retained proof,
  transition admission, and pre-entry execution.
- Delete the measured-rate state dependency from the authority contract.
- Add deterministic unit coverage and update telemetry/documentation.

## Non-scope

- No steering-rate, wall-margin, solver, or V2X parameter tuning.
- No legacy normal-control fallback.
- No relaxation of retained-world or wall certificates.
- The single-vehicle `make dev` empty-V2X evidence contract remains a separate
  upstream issue; dynamic acceptance for this slice uses `make dev2`.

## Evidence boundary

- Baseline: `937bbeb`
- Failing run: `output/20260826-042358`, domains 1 and 2
- First visible failure: repeated `steering-unreachable` retained rejection
  after AWSIM `ready`
- Physical evidence: measured steering rates beyond the model bound were
  clamped and extrapolated for about 0.13 s, moving the alleged physical
  origin by about 0.09 rad away from the measurement.
