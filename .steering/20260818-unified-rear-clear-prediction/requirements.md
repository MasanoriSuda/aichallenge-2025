# Requirements

## Problem

The 20260818-072000 `make dev2` run showed two incompatible live rear-clear
estimates for the same active Pass.  The runtime completion rollout predicted
roughly 19--24 m, while the Pass-horizon extension path independently
recomputed 48--79 m.  The latter kept Pass active across later curves and fed
physical wall revalidation failures.

## Goal

- Use one acceleration-, speed-cap- and execution-policy-coupled runtime
  completion prediction for forward completion, SafeSeparation budgets and
  Pass-horizon extension.
- Remove the second nominal-closing-speed rollout from the horizon path.
- Keep immutable Pass time/distance limits and every wall, target, emergency
  and solver guard unchanged.

## Constraints

- Do not change ROS 2 topics, services, launch files or parameter schema.
- Do not tune wall clearance, speed or acceleration parameters in this change.
- Do not include the user's local `config.yaml` or result JSON changes.

## Acceptance criteria

- A live rear-clear horizon is derived from the already computed runtime
  completion result, including its remaining lateral-transition distance.
- An unavailable, invalid or infeasible runtime completion prediction cannot
  invent a feasible horizon extension.
- Unit tests and the package build pass.
