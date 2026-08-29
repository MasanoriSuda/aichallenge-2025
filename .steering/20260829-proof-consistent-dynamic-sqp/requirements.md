# Requirements: proof-consistent dynamic SQP audit

## Objective

Determine whether the remaining dynamic-proof rejection is caused by keeping
physical obstacle rows frozen while the nonlinear vehicle trajectory changes.
Compare the current single-SQP pipeline with an observation-only bounded outer
SQP which rebuilds dynamics, physical obstacle supports, and physical wall
rows around one common latest primal.

## Constraints

- Production authority and published commands remain unchanged.
- Use the same immutable snapshot, target, side/homotopy, physical geometry,
  solver tolerances, bounds, and exact certificates in both arms.
- Do not add a fallback, lease, grace period, timeout, clearance, or solver
  tolerance change.
- The audit may use only the existing bounded physical-proof correction count.
- A successful QP is not acceptance; unchanged exact wall, dynamic, and
  terminal-successor proofs must all pass.

## Definition of done

- The comparison can run without enumerating the complete A--G population.
- Each outer iteration rebuilds all three coupled model surfaces from the same
  primal and records its count/outcome.
- Representative frozen failures are classified as convergence, candidate,
  certificate mismatch, or physical infeasibility.
- No production promotion occurs until frozen evidence and regression tests
  justify deleting the old approximation path.
