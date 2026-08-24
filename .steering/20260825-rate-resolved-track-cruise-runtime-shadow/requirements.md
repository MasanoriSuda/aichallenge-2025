# Requirements

## Objective

Connect the isolated six-state steering-rate MPCC to the established Track/Cruise
problem semantics as a latest-only runtime shadow. Obtain solver availability,
latency, constraint and 40 Hz actuation-sampling evidence before any authority
migration.

## Repaired invariant

The existing five-state controller treats curvature as a coarse-stage input,
while the publisher consumes a steering command every 40 Hz cycle. A shadow
candidate used to evaluate the replacement must therefore own steering as a
state and steering rate as its only lateral input. It must never be interpreted
as an executable five-state solution.

## Scope

- Reuse the exact Track/Cruise five-state state/input references, hard boxes,
  weights, linear progress reward and stage timing.
- Remove the legacy first-curvature one-publication-step intersection only from
  the six-state shadow request; steering state plus steering-rate input becomes
  the single actuator reachability owner there.
- Solve in a one-running/one-pending latest-only worker.
- Record immutable source identity, sequence, result age, solver timing,
  constraint provenance and the first bounded 40 Hz steering sample.

## Explicit non-scope

- No production authority, final command, fallback or Recovery change.
- No parameter/configuration change or new feature flag.
- No Follow, Overtake or Rejoin connection.
- No physical wall-certificate promotion; this Slice first establishes
  numerical/runtime viability.
- No warm-start transport until exact six-state progress rebase provenance is
  specified and tested.

## Preserved user state

`aichallenge/result-summary.json` is a pre-existing user change and must not be
edited, staged or committed.

## Rollback

Rollback target: `4285cbf`.
