# Design

## Root cause

The old pipeline solved a broad QP, refined wall bounds around that
provisional progress, solved the wall-only QP, and then appended obstacle
rows. Progress-aligned wall refinement selects a wall profile segment per
stage and narrows each progress box to that segment. A later stay-behind row
therefore cannot trade progress for separation even when braking is the
physically correct solution.

## Coupled sequence

1. Solve and relinearize the broad current-world problem.
2. Build and solve the wall-refined problem as a physical homotopy witness.
3. Classify the dynamic-obstacle disjunction from that witness.
4. Apply the selected obstacle rows to the broad relinearized problem, not to
   the witness's narrow progress boxes.
5. Solve the dynamic-aware provisional problem.
6. Build progress-aligned and swept-footprint wall constraints around that
   dynamic-aware trajectory while retaining the obstacle rows.
7. Solve the joint wall-and-obstacle problem.
8. Publish only after the existing exact physical proof accepts it.

The sequence is a bounded SQP/refinement ordering, not a retry fallback.
When either wall or dynamic refinement is inactive, the existing single-layer
path remains unchanged.

## API boundary

`mpcc_rate_resolved_dynamic_obstacle::Request` separates:

- `wall_only_problem` and `wall_only_primal`: physical witness used to choose
  the disjunct/homotopy;
- `constraint_target_problem`: compatible problem that receives the generated
  rows.

Compatibility requires identical horizon and state/input dimensions. The
dynamic constraints of the target are replaced atomically.
