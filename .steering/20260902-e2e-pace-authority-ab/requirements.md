# Requirements

## Objective

Determine whether the current approximately 90-second clear lap is limited by
the frozen lateral policy or by its fixed `0.6 m/s2` longitudinal request.

## Frozen authority

- raw TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- spatial production SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- recurrent authority remains disabled
- `fixed_lidar_brake` remains the longitudinal safety owner

## Evidence motivating the experiment

The admitted single-vehicle baseline has mean speed `3.379 m/s`, maximum speed
`4.545 m/s`, and zero longitudinal-safety interventions in `6542` scans.  The
approximately 90-second lap therefore is not caused by the obstacle brake in
that run.

## Constraints

- The packaged default remains `0.6 m/s2` until a complete Gate is accepted.
- Acceleration experiments must have explicit runtime provenance.
- Test `0.8 m/s2` before `1.0 m/s2`; do not skip a failed lower bound.
- Do not modify steering weights, steering bounds, obstacle distances, or
  recurrent authority in the same experiment.
- A single-vehicle candidate must finish three laps with zero penalty, stall,
  stale sensor or inference error before it may enter an NPC Gate.
- NPC non-regression is required before changing the packaged default.
