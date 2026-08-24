# Requirements

## Objective

Preserve independent linear MPCC objectives, notably stage and terminal
progress reward, when constructing the rate-resolved shadow QP.

## Invariants

- Linear objective terms remain distinct from quadratic references.
- An absent linear vector means exact zero for backward compatibility.
- A supplied vector must have exact primal dimension and finite values.
- The adapter maps all five legacy state and three legacy input linear terms;
  steering and steering-rate terms remain explicitly zero unless supplied by a
  future semantic contract.
- No runtime linkage, authority, parameter or fallback change.

## Definition of Done

- QP assembly combines linear terms with quadratic references exactly.
- Invalid dimensions and non-finite terms fail closed.
- Adapter mapping is covered by deterministic tests.
- Build, full package tests and production non-link audit pass.
