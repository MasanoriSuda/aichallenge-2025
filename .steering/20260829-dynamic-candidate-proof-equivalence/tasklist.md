# Task list: dynamic candidate/proof equivalence

- [x] Freeze scope and prohibit parameter/fallback changes.
- [x] Run existing A--G comparison on representative frozen failures.
- [x] Trace scalar separation and physical geometry provenance.
- [x] Quantify node-geometry, swept-segment, and integration mismatches.
- [x] Implement an observation-only proof-consistent row comparison.
- [x] Decide whether a production replacement is justified.
- [x] Add focused tests and remove the replaced production row path.
- [x] Replay representative frozen failures and the focused regression corpus.
- [x] Build the affected targets and record results.
- [x] Commit the completed Slice.

Next Slice: keep production authority unchanged and compare a bounded outer
SQP that rebuilds both nonlinear dynamics and physical dynamic-obstacle
supports around the latest primal.  Do not increase solver tolerances or add a
new fallback.  Consider inter-sample rows only after nonlinear stage nodes are
clear.
