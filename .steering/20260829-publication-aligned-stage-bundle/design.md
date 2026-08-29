# Design: publication-aligned stage bundle

## Root contract

The publisher holds one serialized command for 25 ms.  A source artifact
stage may end before that interval because its 100 ms stage grid is not phase
aligned with the first asynchronous publication cursor.  Such a residual
cannot own the next command.

The current implementation sends the residual cursor to the physical adapter,
which correctly rejects it.  The missing operation is upstream: choose a
publisher-executable source stage before extracting actuation.

## Repair

Keep two cursors:

- source cursor: the exact artifact time used to compare the current vehicle
  with the source course/progress cross-section;
- command cursor: the control stage whose command can remain active for at
  least one complete publisher interval.

If the source stage has a full interval remaining, both cursors are identical.
If not, advance only to the immediately following stage when that stage has a
full interval.  The skipped residual receives no authority.  Extract the next
stage command, join it to the actually serialized predecessor, and run the
unchanged nonlinear continuation, wall, dynamic, Follow and terminal proofs
from the fresh control-origin state.

Any accepted advance is a stateless current-world Bundle.  The source artifact
remains homotopy/control provenance and is not promoted as the executed plan.

## Alternatives

- Retain Emergency at every boundary: safe but creates deterministic braking
  holes and is the observed defect.
- Change solver stage duration or publisher rate: parameter masking; phase can
  still drift and the causal contract remains wrong.
- Interpolate/blend adjacent controls: creates a command absent from the sealed
  artifact and requires a new optimizer/certificate.
- Advance to the next sealed stage and re-prove: selected.  It changes no
  limits and gives the exact issued command a current-world certificate.

## Exit classification

- A, persistent exact-time artifact: fails with `invalid-cursor`.
- B, stateless publication-aligned Bundle: expected to succeed.
- C, new spline/lattice candidate: unnecessary for the frozen snapshot.
- D, offline nonlinear solve: unnecessary if B passes the unchanged proof.

This classifies the failure as a scheduling/lifecycle connection defect, not
physical infeasibility or candidate-generation failure.
