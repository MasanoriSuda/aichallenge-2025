# Requirements

## Objective

Keep canonical Overtake execution receding: an accepted active ShiftOut, Pass
or Return must receive a current-world canonical candidate from the same
updated tactical path, rather than depending on an aging immutable plan until
physical proof rejects it.

## Failure-first evidence

Run `output/20260824-145739` separates two conditions which must not be merged:

- some Idle/pre-entry locations reject every geometric candidate with
  `static wall physical footprint infeasible`; staying on Follow may be valid;
- after ShiftOut is admitted, rolling DP refresh succeeds at waypoint 13, but
  the canonical worker reports `extended MPCC lateral tracking tube unavailable
  at state 18`, fresh production stops, and an approximately `1.75 s` old plan
  reaches `initial-corridor-violation`.

## Repaired invariant

For an active canonical Overtake intent, each accepted tactical refresh must
either produce a matching current-world canonical plan for a physically proven
receding horizon, or expose one typed infeasibility reason before the old plan
becomes the only normal candidate. Tactical path refresh and canonical plan
production may not silently disagree about their execution horizon.

## Constraints

- Do not weaken physical wall/current-world proof or the 0.15 m tracking tube.
- Do not extend plan age, grace, retry, lease or fallback.
- Do not add a legacy normal command owner.
- Do not tune wall margin, solver settings or Overtake thresholds.
- Do not treat course locations with genuinely infeasible pre-entry geometry as
  the same defect as active Mission producer discontinuity.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- The exact first canonical producer failure includes stage, physical bounds,
  width, requested reserve and intent.
- Tactical and canonical horizons have one explicit, testable ownership rule.
- An executable bounded prefix is never mislabeled as a complete Mission.
- Active Overtake does not reuse an old plan solely because a farther unchecked
  or infeasible tail was appended to an otherwise proven prefix.
- Source contracts, focused tests, package tests, build and bounded `dev2` pass.
