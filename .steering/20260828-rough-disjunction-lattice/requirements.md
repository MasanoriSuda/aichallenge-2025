# Requirements: Rough disjunction lattice candidate C

## Objective

Evaluate architecture candidate C on frozen interaction fingerprint
`7246006054995400977`: enumerate rough stay-behind-to-full-side transition
paths and refine each with the unchanged seven-state SQP and exact proofs.

## Repaired comparison gap

Candidate B delegated branch timing to the wall-only witness.  That producer
created nine partial-escape rows which did not certify either member of the
physical behind-or-side disjunction.  Candidate C must choose branch timing
explicitly and may not use a partial-separation row.

## Constraints

- Production command authority and live default behavior remain unchanged.
- Physical wall/opponent models, solver settings, weights, clearances,
  tolerances, horizon and terminal semantics remain unchanged.
- Every lattice member has an immutable fingerprint.
- Every accepted result must pass the same exact wall and timed dynamic proof.
- No result is publishable; this is architecture evidence only.

## Definition of Done

- A failure-first test proves the current producer cannot force an exact
  behind-to-side transition without partial escape.
- Candidate C enumerates both homotopies and every valid transition stage.
- The frozen snapshot is replayed and C is classified.
- Focused tests, full package tests and `make autoware-build` pass.
- The result and next decision are recorded in the experiment registry.
