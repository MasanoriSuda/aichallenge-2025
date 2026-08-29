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
candidate or four SQP corrections. The QP stage model advances one complete
stage per temporal Frenet transition, while the exact certificate replays the
same held command in substeps of at most 10 ms. Repeating SQP around the coarse
stage map therefore does not make it identical to the certificate's finer
discrete map.

This is a stronger hypothesis than "increase tolerance": the optimizer and
certificate currently prove two different numerical dynamics. The next audit
must compare a substep-consistent transition inside the same immutable QP and
physical rows. It remains observation-only until it demonstrates exact proof
closure without changing clearances or tolerances.

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
