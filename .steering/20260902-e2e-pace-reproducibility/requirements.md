# Requirements

## Objective

Determine whether the newly qualified `0.8 m/s2` packaged acceleration is a
repeatable NPC result rather than a favorable seed-specific outcome.

## Frozen production identity

- TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- spatial adapter SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- packaged acceleration: `0.8 m/s2`
- control mode: `fixed_lidar_brake`
- recurrent authority: disabled

## Constraints

- Do not override acceleration; exercise the packaged default.
- Do not change steering, obstacle distances, braking or checkpoints.
- Use seeds `2034` and `2035`, which are outside the current recorded closed-loop
  evidence set.
- A seed passes only with 3/3 laps, zero penalty, zero stall, zero stale scan and
  zero inference error.
- A failure is frozen and diagnosed before any implementation change.
