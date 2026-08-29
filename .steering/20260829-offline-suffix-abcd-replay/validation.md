# Validation

## Inventory

There are 143 generated `snapshot.yaml` files across the current workspace
locations. Complete v2 interaction snapshots with a warm primal are available
for ShiftOut, Follow and Cruise. Existing Pass and Return artifacts are v1
exact-QP captures and correctly fail the interaction loader because they do
not own the required world provenance. No generated snapshot was edited or
staged.

## Representative 25 ms probes

### ShiftOut physical-proof failure, sequence 1266

- A old-origin: solved, exact proof rejected;
- B common-clock suffix: solved, exact proof rejected;
- C reachable candidate: solved, exact proof rejected;
- D four SQP solves: all solved, final exact proof rejected;
- exact reason: `invalid-lateral-bounds`, dense stage 339;
- C lateral/upper: `1.76053 / 1.75826 m`.

Classification: `solve-proof-model-mismatch`.

### Follow physical-proof failure, sequence 531

- A/B/C solved, exact proof rejected;
- D completed four solves and exact proof still rejected;
- exact reason: `invalid-lateral-bounds`, dense stage 473;
- C lateral/upper: `1.11689 / 1.11687 m`.

Classification: `solve-proof-model-mismatch`.

### Cruise dynamic-obstacle solve failure, sequence 601

- A/B/C all rejected numerically at the configured 4000 iterations;
- D cannot start another correction because solve one has no accepted primal;
- no physical artifact exists.

Classification: `suffix-family-unresolved`.

## Root-cause consequence

The two exact-proof cases are not fixed by a common clock, a reachable initial
candidate or four SQP corrections. Inspection initially suggested different
stage integration, but that hypothesis is refuted: both the full-stage SQP
transition and the exact proof use the same midpoint integration with substeps
no longer than 10 ms.

The remaining representation gap is inside each stage's wall certificate.
The QP constrains stage endpoints plus four rows whose lateral state is the
affine interpolation of the two optimized endpoints. The exact proof instead
checks the true nonlinear lateral state every 10 ms against interpolated wall
bounds. Repeating SQP improves endpoint dynamics but does not add those true
nonlinear interior wall constraints, so the same substage proof failure can
remain after four solves.

This is still stronger evidence than "increase tolerance". The next audit must
compare a candidate whose SQP contains linearized nonlinear interior-wall
states, while keeping the exact same wall intervals, dynamics, clearances and
physical proof. It remains observation-only until it closes the proof without
loosening a bound.

## Limitation

The latest state/input in this tool is deterministically interpolated from the
recorded warm primal. It is not claimed to be the historical measured vehicle
state. The result classifies the frozen formulation at that probe. Exact live
state capture remains a separate platform improvement.

## Verification

- focused reachable-bridge diagnostics: 5/5 passed;
- `make autoware-build`: 25 packages passed;
- package regression: 54/54 test targets passed;
- `git diff --check`: passed.
