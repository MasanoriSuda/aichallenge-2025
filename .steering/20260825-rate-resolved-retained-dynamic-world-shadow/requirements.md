# Requirements

## Objective

Replace the empty-only dynamic-world proxy in the retained steering-rate
Track/Cruise shadow with an atomic, current all-peer V2X observation and a
time-aligned physical non-overlap proof.  The result remains shadow-only.

## Root-cause constraint

`active_vehicle_count == 0` is not the physical condition required for
retained execution.  In a multi-vehicle race it rejects a vehicle behind or
far away even when that vehicle cannot intersect the retained suffix.  At the
same time, treating missing V2X as empty would hide an unobserved world.

## Required invariants

- One mutex acquisition snapshots message identity, timestamps and every
  vehicle from the same V2X array generation.
- No message, stale/invalid message, mixed generation, jump or invalid motion
  estimate remains fail-closed.
- Every current peer is conservatively inflated using the existing vehicle
  radius, prediction margin and covariance policy.
- The delay prefix, current connector and every remaining retained segment
  are checked against every predicted peer in world coordinates.
- Prediction time starts from the observation age and follows exact retained
  stage timing; no distance-proportional time surrogate is used.
- Static-wall, intent, progress and actuation proofs from the previous Slice
  remain mandatory.
- A vehicle may be present and still be accepted only when the complete
  checked trajectory is physically non-overlapping.
- Missing single-vehicle V2X data is not inferred to mean an empty world.
- No command, publisher, production authority, tuning, timeout, lease or
  fallback is added.

## Definition of Done

- Unit tests cover current clear peers, collision, stale/mixed/invalid input,
  multiple peers and no-data rejection.
- Source contract proves the dynamic proof remains shadow-only and consumes
  one atomic V2X snapshot.
- `make autoware-build` and package tests pass.
- A short `make dev2` run records accepted and rejected dynamic-world reasons
  without a callback-overrun regression.
