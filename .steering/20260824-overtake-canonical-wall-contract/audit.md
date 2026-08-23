# Audit

## Observed mismatch

The production exact Overtake certificate passes
`problem.progress_execution_required_wall_clearance_m` to
`solved_mpcc_execution_path_wall_safe`.  The fresh canonical shadow passes a
literal `0.0`.  Retained current-world revalidation receives only the physical
footprint and therefore also proves zero additional wall clearance.

The mismatch is upstream of authority selection.  Promoting the current
canonical result would let a weaker safety certificate replace the production
path and would invalidate the live Gate evidence.

## Competing hypotheses

1. **The clearance mismatch is only theoretical.** Refuted if a failure-first
   geometry is accepted at zero and rejected at the configured clearance.
2. **Correct clearance makes canonical Overtake unavailable in practice.**
   Confirmed if rebuilt dynamic runs repeatedly reject otherwise complete
   candidates specifically at physical wall proof.
3. **Canonical coverage remains sufficient under the correct contract.**
   Confirmed only by current-world fresh/retained selections in rebuilt dynamic
   evidence with the required clearance visible.

No authority promotion is part of this Slice.

## Root cause conclusion

The mismatch is real and testable: an occupied cell beside the physical body is
outside the zero-clearance footprint, but enters the laterally expanded
`0.40 m` footprint.  Before this Slice the retained proof accepted both cases.
The fresh call site likewise passed a literal zero instead of the problem's
certified requirement.

The correction did not make canonical Overtake unavailable in the live Gate.
In `output/20260824-072942`, all 74 fresh solutions which reached physical
proof passed it with `wall_clearance=0.400m`; 71 cycles also completed
current-world fresh/retained selection.  The remaining five cycles were two
async/context-pending samples and three typed current-world corridor rejects,
not wall-clearance rejects.

During the same episode production still emitted eight
`legacy-normal-bypass` decisions.  Its converted path later failed exact wall
proof at stage 11 and moved ShiftOut to FollowPrepare.  Therefore the corrected
certificate is viable, while the persistent next defect is the split Overtake
authority—not insufficient wall margin or canonical wall-proof coverage.
