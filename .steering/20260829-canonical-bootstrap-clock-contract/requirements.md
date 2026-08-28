# Requirements: canonical bootstrap clock contract

## Objective

Restore cold-start canonical Track/Cruise authority without weakening any
actuator, wall, dynamic-obstacle or exact-trajectory proof.

## Root cause

`UnpublishedCandidate` currently has two incompatible meanings:

1. the first certified artifact, for which no predecessor plan has ever been
   published and no artifact prefix has executed;
2. a moving successor produced while another certified artifact continues to
   execute.

The first must start at artifact cursor zero. The second must use a
time-aligned suffix and prove its connector from the actually published
predecessor. Treating both as elapsed suffixes made cold start request steering
from an unexecuted prefix and created a self-sustaining Emergency Stop loop.

## Invariants

- one canonical seven-state normal authority remains;
- no fallback, lease, timeout, grace period or parameter change;
- a bootstrap artifact is allowed only when the Store has no executed plan;
- a moving unpublished successor never resets to cursor zero;
- published plans continue from their exact atomic publication origin;
- all current-world actuator, wall, obstacle, Follow and exact proofs remain
  unchanged.

## Dynamic gate

- `make dev2` must publish a canonical Cruise command from a bootstrap
  candidate and both vehicles must leave zero speed;
- the first published plan must subsequently use the PublishedPlan clock;
- Overtake testing resumes only after this independent startup gate passes.
