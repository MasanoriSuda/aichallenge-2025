# Requirements

## Problem

The frozen outer-role handoff is admitted correctly, but the runtime handoff
reuses the admission-time lateral shift distance as a hard maximum.  In run
`20260805-230303`, the mission stored a 1.35 m ramp while ego was near
1.39 m/s.  At the scheduled window ego was approximately 3.2--3.7 m/s, so all
three attempts were rejected with `same-side lateral ramp exceeds acceleration
budget` despite almost 8 m of transition window remaining.

## Required behavior

- Keep the admitted opposite-side lateral goal frozen.
- Treat the admission-time shift distance as a nominal plan, not the runtime
  maximum.
- Size the admission ramp using at least the candidate overtake command speed.
- At execution, recompute the required ramp from current ego speed and allow it
  to consume the remaining scheduled window, bounded by the configured maximum.
- Re-run the existing rollout, wall, lateral-acceleration, target-front and
  Return preflights before committing the replacement path.
- If the ramp still cannot fit, log required and available distances without
  changing the safety fallback.

## Constraints

- Do not change ROS topics, message types, launch contracts or evaluation code.
- Do not loosen wall, footprint, V2X freshness or target longitudinal guards.
- Do not change global acceleration or braking parameters.

