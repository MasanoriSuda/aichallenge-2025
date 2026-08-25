# Requirements

## Objective

Promote Follow to the same `velocity-steering-progress-6state` normal authority
used by Track, Cruise, ShiftOut, Pass, and Return, and physically remove the
retired five-state Follow normal owner in the same Slice.

## Root architectural reason

Follow currently has a dedicated five-state solver, retained store, worker,
transition admission, and publisher path.  Normal intent changes therefore
still change formulation and lifecycle ownership.  This violates the
single-authority invariant even when both implementations are individually
certified.

The shared extended problem builder already expresses Follow progress bounds,
target progress, velocity reference, velocity hard cap, and gap rows.  The
rate-resolved adapter consumes those exact state/input rows.  Follow therefore
needs an authority migration, not a second set of tuning parameters.

## Scope

- Extend the six-state artifact and production scope to Follow.
- Route Follow dispatch through the shared rate-resolved production owner.
- Preserve Follow longitudinal and dynamic-obstacle current-world proof.
- Delete the old Follow five-state lifecycle, worker, store, transition
  admission, telemetry, and publisher call path once the new owner is wired.
- Add source-contract and focused behavior tests before production changes.

## Prohibited changes

- No gap, speed, wall, solver, weight, horizon, timeout, or cadence tuning.
- No feature flag, compatibility fallback, grace period, or dual publication.
- No restoration of a five-state normal command after a six-state rejection.
- Do not modify or commit `aichallenge/result-summary.json`.

## Exit gate

- Follow final contracts report `velocity-steering-progress-6state`.
- A fresh Follow transition is atomically admitted by the same six-state
  producer and ordinary retained current-world proof as other normal intents.
- Missing or rejected six-state evidence produces explicit Emergency, not a
  different normal formulation.
- Static search and failure-first tests prove the old Follow owner is absent.
- Build, all package tests, and a bounded `dev2` replay pass.
