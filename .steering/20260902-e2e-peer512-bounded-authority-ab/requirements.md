# Requirements

## Objective

Determine whether the frozen peer-512 recurrent artifact improves closed-loop
peer interaction when granted explicit bounded steering authority.

## Frozen inputs

- production TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- recurrent artifact SHA-256:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`
- recurrent hidden/projection dimensions: `512 / 128`
- recurrent speed input: disabled
- authority correction bound: `+/-0.24 rad`
- authority-disabled A baselines:
  - `/output/20260902-e2e-recurrent-shadow-isolated-single`
  - `/output/20260902-e2e-recurrent-shadow-isolated-peer-v2`

## Constraints

- Keep packaged production checkpoint and default recurrent authority unchanged.
- Authority is enabled only through the explicit experiment environment.
- Do not run peer authority until the single-vehicle race, motion, production
  and recurrent authority Gates all pass.
- Recurrent authority owns only bounded steering correction.  It may not own
  acceleration, safety braking or watchdog behavior.
- Authority-enabled inference remains current-sample synchronous.  Do not
  publish an old asynchronous diagnostic result as a command.
- Do not relax scan frequency, coverage, freshness, reset, penalty or stall
  Gates in response to a failure.
- A passing run is experimental evidence only; it does not package or promote
  the artifact.

## Definition of Done

- The single-vehicle B run is compared with its frozen authority-disabled A
  baseline.
- If single B passes, the deterministic three-vehicle B run is compared with
  the frozen domain-3 A baseline.
- Finish, penalty, stall, lap time, inference timing, coverage, resets,
  authority applications and clipping are reported.
- The candidate is admitted only for a repeat or rejected; production defaults
  remain unchanged in either case.
