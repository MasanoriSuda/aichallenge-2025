# Design: proof-consistent dynamic SQP audit

## Root hypothesis

The current post-refinement correction rebuilds affine vehicle dynamics, but
the dynamic-obstacle half-spaces remain linearized at the earlier wall-only
witness.  The final nonlinear rollout can therefore move its stage pose and
heading away from both the dynamic row tangent and physical support used to
construct that row.

## Audit arm

Add an observation-only `SolverContext` entry point.  After the existing
dynamic/wall solve, perform a bounded outer loop using the already-established
physical-proof correction count:

1. relinearize seven-state dynamics around the latest primal;
2. rebuild the selected dynamic disjunct and oriented body support around that
   same primal;
3. rebuild physical wall rows around that same primal;
4. assemble and solve the unchanged racing objective;
5. repeat only within the fixed audit bound;
6. run the unchanged exact proof chain in architecture comparison.

The snapshot's forced schedule preserves candidate homotopy where one exists.
The audit has no Store, mailbox, publisher, or production caller.

## Classification

- single-SQP rejects, bounded dynamic SQP certifies: single-SQP limitation;
- both reject but another candidate certifies: candidate/homotopy defect;
- affine and nonlinear nodes clear but sweep rejects: inter-sample model gap;
- every bounded candidate and offline nonlinear check reject: physical
  infeasibility.
