# Requirements

## Purpose

Produce a prospective steering-rate-resolved six-state Gate A artifact for
each left/right Overtake branch from the exact same immutable tactical
snapshot used by the remaining five-state Gate A. Keep it shadow-only until
dynamic evidence exists.

## Root cause

The normal publisher is already six-state, but Mission entry is still admitted
by a five-state solve. Deleting that solve without replacement would allow an
unproved Mission to mutate behavior state. Reusing the ordinary production
request builder is also incorrect because it derives intent from the current
live authority (usually Follow), whereas Gate A must prove a prospective
ShiftOut or Pass intent.

## Required behavior

- Build the six-state request from an explicit prospective intent.
- Use one dedicated solver context per left/right homotopy so the concurrent
  worker cannot mix warm or solver state.
- Certify exact static-wall and target-tube clearance for the six-state path.
- Record solver, wall, target, timing and terminal metrics for both sides.
- Keep branch selection, Mission mutation and final command publication on the
  existing Gate A during this Slice.
- Do not use the production retained plan store or final publisher from the
  shadow path.

## Non-goals

- Do not remove the five-state Gate A yet.
- Do not promote a six-state shadow result to authority.
- Do not tune solver, horizon, weights, clearance, timeout or cadence.
- Do not add a fallback, lease, cooldown or feature flag.

## Exit gate

- Failure-first source contracts prove explicit prospective intent and
  commandless shadow ownership.
- Build and full package tests pass.
- A bounded dynamic run records left/right six-state shadow results without
  changing selected Mission or final execution formulation.
- Promotion remains blocked until six-state availability and physical/target
  acceptance are measured against the current Gate A.
