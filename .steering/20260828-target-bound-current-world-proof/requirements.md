# Requirements

## Objective

Prevent a committed Pass from continuing a stale lateral trajectory after the
current dynamic-obstacle certificate has become unsafe.

## Frozen evidence

- baseline: `990192b3`
- run: `output/20260828-132039/d1/autoware.log`
- episode 2 reached `ShiftOut -> Pass`;
- the target-bound hold started at `wp_id=197` after `target[5]` became
  infeasible;
- the target sweep became unsafe while current bodies were still separated;
- the old prefix continued for `1.80 s / 9.51 m`;
- the target then reached `1.51 m` longitudinal and `1.32 m` lateral
  separation, producing current footprint overlap and SafetyBrake.

## Constraints

- Do not tune solver limits, clearance, lease time or speed parameters.
- Do not add another fallback, timeout or grace period.
- A retained prefix must carry a current dynamic-obstacle certificate.
- A persistent Mission may retain identity/homotopy/commit state, but may not
  retain lateral path samples merely because the Mission is active.
- Keep Emergency Stop ownership unchanged.

## Definition of done

- Target-bound repair cannot reuse an old moving lateral trajectory.
- Every admitted target-bound prefix requires a valid, separated current
  target sweep (except separately qualified recoverable contact).
- Obsolete target-bound solved-prefix state/configuration is removed.
- Focused/full tests and build pass.
- A new `make dev2` run no longer travels through an unsafe target sweep on a
  retained Pass prefix.
