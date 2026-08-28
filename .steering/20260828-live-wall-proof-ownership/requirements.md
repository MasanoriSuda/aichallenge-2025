# Requirements

## Objective

Remove duplicate footprint-aware wall-corridor construction from the live
OvertakeLine reference planner.  The canonical seven-state latest-only worker
must remain the sole owner of optimized-trajectory wall constraints and exact
swept-footprint certification.

## Frozen evidence

`output/20260828-212704` reproduced three `update_overtake_line()` calls above
20 ms during `ShiftOut`:

- decision 1429: 38.599 ms total, 38.041 ms receding optimization, 32 wall
  cache misses and 4308 scanned poses;
- decision 1494: 26.522 ms total, 22.087 ms receding optimization, 19 wall
  cache misses and 2563 scanned poses;
- decision 1544: 22.431 ms total, 21.425 ms receding optimization, 20 wall
  cache misses and 2633 scanned poses.

The live reference optimizer requested both hard and preferred physical wall
intervals for every one of the 20 stages.  The selected trajectory was then
submitted to the seven-state worker, which independently applied the same map
and footprint as hard refinement constraints and ran an exact swept-footprint
certificate before publication.

## Constraints

- Do not change wall or opponent clearances.
- Do not change solver tolerances, cadence, horizon, leases or fallbacks.
- Do not change production authority or state-transition policy.
- Retain the inexpensive exact physical validation of the generated reference
  path; it remains a fail-closed Mission viability check.
- Preserve async physical refinement and final exact certification.
- Do not commit generated run artifacts or user result files.

## Exit criteria

- `optimize_live_overtake_line_horizon()` performs no footprint wall-envelope
  search.
- Its wall bounds are explicitly treated as scalar reference support, not as a
  physical certificate.
- The seven-state latest-only worker still receives the wall map, expanded
  footprint and progress-indexed support and still requires exact proof.
- Build and package tests pass.
- A bounded `make dev2` run confirms that `OvertakeLine` no longer owns the
  20-stage physical wall-envelope scan and checks for behavioral regression.
