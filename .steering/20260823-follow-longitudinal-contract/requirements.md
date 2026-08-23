# Follow longitudinal MPCC contract requirements

## Purpose

Begin Slice 4 without promoting a new production authority. Replace the implicit
"one scalar Follow speed cap copied to every stage" assumption with a typed,
stage-wise longitudinal contract that a canonical five-state MPCC can consume.

## Repaired invariant

For a Follow intent, the target identity, observation generation and freshness
used to build the longitudinal horizon must be explicit. Every horizon state
must name both:

- the desired ego progress that preserves the configured moving-follow target
  centre distance; and
- the maximum ego progress that preserves the configured hard centre distance.

A retained behavior label is not a target observation. Shadow eligibility
therefore also requires a finite, current front-vehicle observation whose
target provenance matches the selected target in the same behavior cycle.

## Scope

- Pure contract construction and deterministic failure reasons.
- Constant-along-course target-speed prediction from the current fresh V2X
  observation.
- Existing configured target/hard distances and speed-margin policy; no new
  tuning value.
- Unit tests for moving, stopped, opening-gap, stale/disappearing and malformed
  observations.
- A typed rejection for Follow labels that have no coherent front observation.
- Shadow-only integration preparation. No publisher/authority change.

## Non-scope

- Follow production promotion or deletion of the legacy scalar cap.
- Hold/Stop integration.
- Opponent acceleration/tube uncertainty beyond the currently available fresh
  longitudinal observation.
- Overtake, Dynamic Escape, Recovery or parameter tuning.

## Failure-first acceptance

The contract must reject before solve when target identity/generation,
freshness, horizon time, configured distances, speed or current hard gap is
invalid. It must never repair an infeasible current hard-gap state by clamping
the first progress bound.
