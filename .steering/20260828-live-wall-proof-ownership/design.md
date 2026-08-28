# Design

## Root cause

The live callback had two different physical-wall owners:

1. `optimize_live_overtake_line_horizon()` scanned a heading-dependent
   footprint interval twice per stage and used those intervals to construct a
   one-dimensional lateral reference;
2. the canonical seven-state latest-only worker used the current map and
   expanded footprint to refine the solved trajectory, then ran the exact
   swept-footprint certificate required for publication.

The first computation is neither the published trajectory nor the final
certificate.  Its heading key changes as the receding reference advances, so
the cache cannot turn it into bounded 40 Hz work.  This is duplicated ownership,
not a cache-size or tolerance defect.

## Ownership correction

The live OvertakeLine optimizer remains a **reference generator**.  It may use:

- scalar course bounds;
- the configured preferred/hard wall reserve;
- target prediction and selected homotopy;
- lateral reachability and reference-path physical viability.

It may not construct a footprint-aware feasible corridor.  Its stage bounds
are renamed locally from `*_physical_wall_*` to `*_wall_*` to make the contract
explicit.

The canonical seven-state pipeline remains the **trajectory and proof owner**:

- the progress-indexed scalar wall profile supplies finite first-solve support;
- `bind_rate_resolved_physical_wall_refinement()` supplies the immutable map,
  clearance-expanded footprint and course frame to the latest-only worker;
- worker-side physical refinement creates footprint-aware hard constraints;
- exact swept-footprint proof gates the certified store and publication.

The live reference is still passed through
`evaluate_overtake_line_horizon(..., enforce_execution_feasibility=true)`.  That
check rejects a physically invalid reference but does not claim that its scalar
stage bounds are a physical corridor.

## Rejected alternatives

- Broaden the heading cache bucket: changes geometric conservatism and leaves
  duplicate ownership.
- Increase refresh interval or reuse age: adds stale execution policy.
- Reduce wall clearance or sampling density: weakens the certificate.
- Add another async wall cache: creates a third lifecycle instead of removing
  the redundant owner.

## Dynamic decision rule

- If receding optimization time and wall-cache misses disappear without new
  physical rejects, close the ownership Slice.
- If time moves into canonical problem assembly, audit the remaining generic
  scalar-to-physical profile builder before changing timing parameters.
- If worker physical refinement or exact proof stops being requested, revert
  the Slice: that is a certificate regression, not an acceptable speedup.
