# Design

## Comparison

```text
B: latest x0 + sliced old future states/inputs
   -> linearize directly around the discontinuous joined primal

C: latest x0 + sliced old input/homotopy intent
   -> enforce existing input and cumulative steering bounds
   -> canonical nonlinear seven-state rollout
   -> linearize unchanged full QP around the reachable rollout
```

Both arms keep the same absolute stage clock, source identity, state/input
costs, state/input boxes, wall rows, dynamic-obstacle rows and terminal
successor. Only the numerical tangent candidate differs.

## Reachable rollout

For each surviving stage:

1. take the input from the prepared suffix;
2. project it into the existing input box;
3. additionally project steering rate so the cumulative physical steering
   prefix stays inside its existing certified interval;
4. propagate the exact current state through
   `evaluate_temporal_frenet_transition`;
5. use the resulting state/input sequence solely as the next SQP tangent and
   cold-dual bootstrap.

State boxes are not used to clamp the nonlinear rollout. Box violation is
reported as candidate evidence; the unchanged QP and physical proof decide
acceptance.

## Architecture boundary

The evaluator remains under `LatestStateFeedbackSolverContext`, which has no
Store, mailbox or publisher. Production remains exactly at `7eecb011` while
the comparison is incomplete.
