# Requirements

## Goal

Make the six-state execution artifact preserve and publish the steering sample
for the next command publication boundary, rather than treating the
latency-compensated future physical steering state as an immediate desired
command.

## Evidence boundary

- baseline HEAD: `f5e56c3`
- run: `output/20260826-111752`
- first race-start failure: domain 1, decision 900, sequence 301

The exact transition solve, wall proof and certified plan pass.  Current-world
join rejects only steering continuity:

```text
physical_control_origin=-0.092724
previous_published=-0.117711
expected=-0.092724
publication bounds=[-0.141895,-0.093526]
world=steering-unreachable
```

## Required invariant

The execution artifact has two distinct time meanings:

- `semantic_initial_steering_rad`: physical steering at the latency-
  compensated prediction origin;
- published steering: the certified piecewise-rate sequence sampled one
  publication interval after the current execution cursor.

These must never be substituted for each other.  The publication interval must
be sealed into the immutable artifact and validated against its horizon.

## Constraints

- Do not change steering-rate, delay, horizon, wall or solver parameters.
- Do not clamp a steering command or add a fallback/lease/timeout.
- Keep physical state, desired command history and publication sample as
  distinct values.
- Preserve all ROS/evaluation/submission contracts and the user-owned result
  JSON modification.

## Definition of done

- Failure-first test reproduces a physically valid initial steering that is
  not itself publication-reachable, while its certified next-publication
  sample is reachable.
- Artifact validation rejects missing/invalid publication timing.
- Retained proof and production command use the certified publication sample.
- Values beyond the certified horizon fail closed.
- Focused/full tests and build pass.
- Moving run removes the decision-900 class of transition rejection without
  introducing publication mutation or another authority.
