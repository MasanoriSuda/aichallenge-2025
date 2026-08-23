# Design

## Contract alignment

The production five-state wall certificate expands the vehicle's left and
right footprint extents by the required hard wall clearance and validates the
current-pose-to-horizon swept path.  Canonical Overtake must use that same
scalar and expansion rule for both candidate lifetimes:

1. Fresh solution: pass
   `problem.progress_execution_required_wall_clearance_m` to the existing exact
   swept-wall validator.
2. Retained solution: seal the same scalar into the current-world proof request,
   validate that it is finite and non-negative, expand the lateral footprint,
   then use that one footprint for delay prefix, connector and every retained
   stage.

The required clearance is current-world safety input, not immutable plan
identity: a retained plan must be re-proved against the current problem's
requirement.  It therefore belongs to the current-world proof request, not to
the stored canonical plan.

## Scope boundary

This Slice changes only whether a canonical shadow candidate is certified.  It
does not connect the selector to final control output and does not alter legacy
production behavior.  Production authority promotion remains a later Slice,
conditional on dynamic evidence under this corrected proof.

