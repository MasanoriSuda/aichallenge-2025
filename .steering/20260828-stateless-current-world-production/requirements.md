# Requirements

## Root cause

Frozen decision 3931 proves that restoring the missing target tube does not
repair the persistent Mission geometry.  The retained reference fails the
wall-refined SQP, while a current-world physical-diagonal candidate passes the
same SQP, exact wall proof, exact dynamic proof and terminal-successor proof.

## Objective

Remove persistent Mission path geometry from the production Overtake SQP
input.  Keep only target, selected homotopy and commit state as tactical input.
Rebuild a bounded current-world candidate population inside the existing
asynchronous branch worker and publish only an existing Gate-A-certified
artifact.

## Constraints

- Do not change solver settings, tolerances, wall clearance, timeouts or
  leases.
- Do not add a new command publisher or fallback authority.
- Use at most two topology candidates per side.
- Keep exact wall, exact timed-obstacle and current-world rejoin proofs as
  mandatory authority gates.
- Remove the raw persistent-geometry SQP path at the same integration point.
