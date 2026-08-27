# Design: one physical progress coordinate for dynamic obstacles

## Root cause

The seven-state model contains both virtual progress `theta` and lag
`e_lag`.  The vehicle's physical along-track position is:

```text
s_ego = theta + e_lag
```

Follow's canonical longitudinal contract and physical certificate already use
that expression.  The Slice 6 dynamic-obstacle refinement instead compared
and constrained `theta` alone.  A valid Follow solution with negative lag was
therefore classified as longitudinally overlapping and converted into an
unrequested lateral homotopy.

## Repair

1. Replace the representable obstacle `Progress` row with
   `EffectiveProgress`.
2. Assemble that row as `theta + e_lag`.
3. Use `theta + e_lag` for wall-only branch classification and feasibility
   prechecks.
4. Preserve raw `theta` and add effective progress to telemetry so a future
   coordinate regression is directly observable.
5. Keep lateral branch bounds and all configuration values unchanged.

## Alternatives rejected

- Replenish Cruise indefinitely while Follow fails: masks the invalid Follow
  producer and creates a lease-like authority extension.
- Increase OSQP iterations or relax steering bounds: does not repair the
  wrong coordinate.
- Disable dynamic-obstacle refinement for Follow: removes physical obstacle
  protection instead of making it coherent.
- Add a special-case transition admission fallback: duplicates authority and
  leaves the same defect in steady-state Follow and Overtake pre-pass stages.

## Deletion boundary

The theta-only longitudinal obstacle row is removed from the type system and
assembly path.  No compatibility alias is retained.
