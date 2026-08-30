# Requirements

## Evidence boundary

- Baseline: `ef60702c audit(mpcc): enforce stop longitudinal contract`
- Frozen failure: decision 4017 in `output/20260830-200852`
- Production authority, publisher, Store and configuration remain frozen.

## Objective

Determine whether a bounded bang-bang steering-rate lattice plus the existing
seven-state SQP can produce a certified maximum-braking Stop.  This is the
candidate-generation escape-hatch comparison that must precede adding a fresh
asynchronous Stop solver to production.

## Constraints

- Use the same rebased publisher-boundary state and solver-safe
  maximum-braking velocity schedule as the corrected seven-state audit.
- Enumerate a bounded, deterministic set of positive/negative/hold and
  negative/positive/hold steering-rate switch patterns.
- Every candidate owns a fresh solver context and must pass the unchanged
  exact physical, wall and current-world dynamic proofs.
- Do not seed candidates from the accepted seven-state control sequence.
- Do not add fallback, lease, timeout, tolerance, clearance or authority.

## Exit classification

- lattice accepted: candidate-generation defect; design a bounded production
  artifact source before considering another solver worker;
- lattice rejected, free seven-state accepted: fixed lattice representation is
  insufficient; a fresh asynchronous seven-state artifact is justified;
- both rejected under the corrected longitudinal contract: continue the
  offline multi-SQP/nonlinear feasibility audit.
