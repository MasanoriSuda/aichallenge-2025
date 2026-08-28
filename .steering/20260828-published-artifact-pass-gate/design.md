# Design

## Root cause

There are two execution identities around the Pass boundary:

1. canonical production owns the last command-producing certified artifact and
   advances it from `first_published_control_origin_sec`;
2. the lateral supervisor owns a DP prefix or a projected solved trajectory and
   advances it from Mission distance while rejecting it by original solve age.

The second identity may disappear while the first is still the command being
published.  Pass entry then proves a fallback lateral line rather than the line
which actually owns actuation.  DynamicWait inherits the same gap and can enter
Recovery despite an available executed suffix.

This is a lifecycle/authority identity defect.  Extending a lease or reducing
wall clearance would only hide it.

## Correction

Add a pure published-source resolver beside the existing execution-source
projection.  It:

- validates the exact certified plan and Mission identity;
- advances the artifact with the existing published execution clock;
- lifts measured course progress into the artifact's local path distance;
- returns no source once the immutable execution cursor is exhausted.

The controller resamples that exact certified lateral trajectory onto the
current horizon.  It is a reference only.  Existing current-state reachability,
wall and lateral-acceleration checks decide whether it remains executable.

Reference precedence at the affected boundary is:

1. last actually published certified artifact with matching identity;
2. active DP execution prefix;
3. temporarily promoted solved bridge;
4. current-goal fallback.

This precedence does not promote a new solver result and does not change the
command producer.  It only prevents the supervisor from proving a different
line than production is currently executing.

## Scope

The initial correction is limited to a ShiftOut artifact at:

- the ShiftOut-to-Pass physical entry gate;
- DynamicWait whose origin phase is ShiftOut.

Pass and Return retain their existing lifecycle until separate evidence shows
the same defect.  This keeps the change one-to-one with the frozen failure.

## Rejected alternatives

- Extend the Pass gate lease: retains split identity for longer.
- Lower wall clearance: changes physical policy without proving a geometry
  defect.
- Keep the solved bridge active unconditionally: turns an unpublished/candidate
  projection into pseudo-authority.
- Re-run full retained production evaluation inside the supervisor: duplicates
  dynamic, wall and actuator proof in the 40 Hz callback.
