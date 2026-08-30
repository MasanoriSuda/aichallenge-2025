# Design

Start from the audit-only maximum-braking Stop snapshot.  For each lattice
candidate, integrate a three-arc steering command:

```text
first solver-safe steering-rate boundary
opposite solver-safe steering-rate boundary
zero steering rate
```

The first and second switch stages are drawn from a small horizon-relative
grid, and both initial signs are evaluated.  After the canonical semantic
adapter builds the unchanged seven-state problem, an audit-only solver entry
fixes its steering-rate input rows to the lattice sequence.  Acceleration input
bounds stay physical; future velocity states retain the already-audited
maximum-braking equality.

The SQP therefore refines lateral, lag, heading, response steering and
progress while it cannot silently change the lattice steering command or the
Stop longitudinal law.  Acceptance still belongs to the nonlinear physical
rollout and the existing wall/dynamic certificates.

This Slice is observation-only.  `Arm::SevenStateStopControlLatticeV` has no
command conversion, Store or publisher edge.
