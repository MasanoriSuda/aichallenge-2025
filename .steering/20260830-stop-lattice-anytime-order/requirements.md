# Requirements

## Objective

Reduce live Stop-lattice result age without changing production authority,
solver tolerances, wall/vehicle clearance, Mission lifecycle, or the complete
set of steering-rate schedules.

## Evidence

`output/20260830-222744/d1/autoware.log` showed that certified Stop suffixes
exist in live ShiftOut and Pass states, but difficult epochs attempted up to 68
schedules and returned results as much as 5.3300 seconds old.  The current
population enumerates every positive initial-rate schedule before any negative
initial-rate schedule.  A valid negative `3:6` schedule therefore appeared at
attempt 42 even though the corresponding positive shape was near the front.

## Constraints

- Keep the exact maximum-braking law and certificate chain unchanged.
- Keep the complete legacy schedule set unchanged.
- Add no timeout, grace period, lease, fallback, solver-tolerance or clearance
  change.
- Do not connect the Stop result to Store, publisher or production authority.
- Preserve deterministic replay and immutable source identity.

## Definition of done

- An anytime population contains exactly the same schedules as the legacy
  population, with no duplicate or missing member.
- Initial steering-rate signs are paired for every geometric schedule rather
  than separated into two full half-populations.
- Geometric shapes are ordered by deterministic broad-coverage sampling so an
  early prefix is not confined to one local part of the grid.
- Live telemetry reports selected anytime and legacy ranks.
- Static tests and authority audit pass before another dynamic run.
