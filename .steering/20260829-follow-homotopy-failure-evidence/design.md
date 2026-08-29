# Design: Follow homotopy failure evidence

## Earliest violated audit invariant

The architecture escape-hatch requires candidates to be compared with the
same immutable problem.  The recorder's process-wide key omits physical
homotopy, so a failure on one side masks the other side before it reaches the
offline comparison.

## Change

Extend only the in-process deduplication identity with the effective physical
side:

1. use `execution_side_sign` when the intent owns an execution side;
2. otherwise use `dynamic_obstacle_pass_side_sign`, including Follow;
3. retain the existing intent, pipeline stage and failure outcome fields.

The interaction fingerprint and serialized payload are unchanged.  No Store,
mailbox, solver or publisher path is added.

## Next classification

For a captured failed negative candidate:

- exact fresh replay succeeds: persistent numerical-context defect;
- exact fresh replay fails but a rebuilt opposite side succeeds: candidate or
  homotopy selection/lifecycle defect;
- solve succeeds but exact proof fails: model/certificate mismatch.
