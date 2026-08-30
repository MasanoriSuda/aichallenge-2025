# Requirements: exact wall free-cell broad phase

## Objective

Remove the dominant exact swept-footprint wall-proof CPU tail without
weakening the wall certificate or changing production authority.

## Frozen evidence

- Baseline: `91d150fc`.
- Run: `output/20260831-013511`.
- d2 decision 3426 spent `12.917 ms` in continuation wall proof and
  `5.152 ms` in terminal Stop wall proof.
- Dynamic-obstacle proof in the same decision took only `0.227 ms`.

## Constraints

- Keep the same occupancy grid, inflated footprint, swept interpolation and
  occupied/unknown-cell semantics.
- No clearance, horizon, solver, Mission, Stop, lease or authority change.
- The optimized evaluator must return the same result and contact-cell set.
- Do not introduce a mutable cache into the control callback.

## Definition of done

- Free cells are rejected before oriented-box/cell intersection work.
- Existing wall-proof tests and all package tests pass.
- A bounded dev2 run measures the same typed wall-proof regions.
- Root fix is accepted only if wall-proof tail falls without new wall
  acceptance or authority behavior.

## Result

- Build succeeded and all 2,248 tests passed.
- Comparison run: `output/20260831-014111`.
- Mean of the two-second retained-evaluation window averages:
  - d1: `2.539 -> 2.241 ms` (about 12% lower)
  - d2: `4.837 -> 3.887 ms` (about 20% lower)
- Exact results remained accepted/rejected through the existing certificate
  path; no authority or clearance setting changed.
- A single d2 cycle still spent `26.479 ms` in continuation wall proof.
  Therefore cell-first rejection is a valid improvement, but AABB traversal
  itself remains the next measured bottleneck.
