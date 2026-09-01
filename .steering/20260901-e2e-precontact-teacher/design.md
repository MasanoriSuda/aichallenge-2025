# Design

## Why this is not threshold tuning

The old teacher computes a 10th percentile over the whole extreme-side sector.
A vehicle occupies only a small angular cluster, so distant background returns
dominate the percentile.  Lowering `side_trigger_distance_m` cannot make that
aggregate represent the nearest physical vehicle.  Raising the trigger can
instead create false reactions to walls.

The new teacher changes two structural operators while retaining the same
physical distances:

1. Side clearance is the Nth-nearest return in the extreme-side sector.  With
   N=3, a single or double bad ray is ignored, while a coherent vehicle/wall
   surface is retained.
2. Side avoidance is a safety projection.  Once a supported side threat is
   active, steering toward that threat is outside the admissible set; it is not
   averaged back in from the ML base command.

## Runtime isolation

```text
fixed_lidar_brake      production, unchanged
gap_teacher            historical teacher, unchanged
precontact_teacher     new diagnostic teacher
```

The diagnostic final-world target keeps d1--d3 in production and assigns only
d4 to `precontact_teacher`.  No production authority changes in this slice.

## Acceptance

Static correctness is necessary but insufficient.  The candidate must:

- activate before 129.65 s in offline replay of the frozen d4 failure;
- command away from the supported right-side threat;
- avoid a run-level positive-acceleration stall longer than 5 s in the
  unchanged final world;
- avoid introducing a new stall on d1--d3 (which remain production controls).
