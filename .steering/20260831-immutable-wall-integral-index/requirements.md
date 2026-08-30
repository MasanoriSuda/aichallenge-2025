# Requirements: immutable wall integral index

## Objective

Eliminate the remaining exact wall-proof AABB traversal tail while preserving
the exact oriented-footprint certificate.

## Frozen evidence

- Baseline: `391480fd`.
- `output/20260831-014111` shows lower mean retained proof cost after free-cell
  geometry rejection, but a `26.479 ms` continuation wall-proof outlier.
- The remaining work still visits every cell in every swept footprint AABB.

## Constraints

- The index is derived once from the final immutable occupancy snapshot.
- A wholly free AABB may be accepted by the broad phase; any AABB containing
  non-free cells must run the unchanged exact geometry.
- Grid fingerprint and exact proof semantics must not include or depend on
  mutable cache state.
- No clearance, swept step, horizon, solver, Mission, Stop or authority change.

## Definition of done

- O(1) non-free AABB query is available only for explicitly prepared grids.
- Production owns and shares one const prepared wall snapshot.
- Indexed and unindexed rasterization are equivalent in tests.
- Build and all tests pass.
- Bounded dev2 shows wall-proof runtime and no proof-semantic regression.

## Result

- Build succeeded and all 2,250 tests passed.
- Dynamic run: `output/20260831-015209` (about 70 seconds).
- No `Canonical production join runtime` warning above 20 ms occurred.
- Mean of retained-evaluation window averages versus `391480fd`:
  - d1: `2.241 -> 0.716 ms`
  - d2: `3.887 -> 1.211 ms`
- Maximum retained-evaluation window values were `6.905 ms` (d1) and
  `8.102 ms` (d2), versus `19.284/32.305 ms` before the integral index.
- Overtake reached `ShiftOut -> Pass -> Return`, then later hit an actual wall
  intersection.  This is not a runtime-tail regression; it exposes the next
  independent model/certificate/execution consistency defect.
