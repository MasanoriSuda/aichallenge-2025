# Requirements: Dynamic certificate mismatch localization

## Objective

On frozen interaction fingerprint `7246006054995400977`, identify the first
physical sample where stateless positive-side B disagrees with its sparse
seven-state obstacle model.

## Constraints

- Production authority and numerical formulation remain frozen.
- No clearance, tolerance, iteration, fallback, lease or timing change.
- Diagnostics must originate in the common exact proof, not infer failure
  timing from the final minimum alone.
- The same frozen snapshot is rerun after observation is added.

## Definition of Done

- Dynamic proof reports rejection reason, elapsed time, pose and obstacle at
  the first failed dense sample.
- The sample is mapped to control prefix or solved trajectory and its enclosing
  QP stage interval.
- The earliest shared-model invariant violation is documented before any fix.

