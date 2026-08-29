# Design: canonical wall-refinement evidence cache

## Root-cause chain

```text
current-world Follow problem
  -> selected homotopy seven-state SQP
  -> quantized progress/lag/heading wall scan for every horizon stage
  -> same static grid/bucket queried with slightly changing lateral bounds
  -> worker completion later than the predicted control origin
  -> solved candidate no longer reachable from serialized steering
  -> executed artifact exhausts
  -> Emergency Stop
```

The cache changes only the repeated middle operation.  The QP rows produced
from a hit are identical to a fresh scan, and the final nonlinear swept-wall
proof still runs for every newly solved artifact.

## Ownership

An initial exact-key prototype was dynamically falsified in
`output/20260829-121626`: runtime telemetry stayed at `cache_hits=0` because
receding lateral bounds changed between every request. It is not accepted.

Each `SolverContext` owns a bounded `wall_refinement::Cache`.  Follow negative
and positive homotopies already own separate solver contexts, so cached
evidence cannot cross their numerical continuation ownership.  Cache keys
include:

- immutable occupancy-grid fingerprint;
- guarded footprint extents and margin;
- exact quantized reference pose and heading offset;
- sample step and boundary guard.

Values contain the exact clear runs plus the lower/upper lateral range already
searched. A subset query selects its interval from those same runs. A wider
query rescans the union before replacing the entry. A hit does not claim that
a candidate is safe; it only removes a repeated immutable raster scan.

## Bounded storage

The cache uses deterministic least-recently-used eviction.  Eviction changes
only compute cost: the next request is rescanned and cannot change proof
semantics.

## Telemetry

Wall-refinement results record cache hits and misses.  Solver diagnostics
append these counters so dynamic A/B can attribute latency reduction without
adding a second control path.

## External cross-check

The ETH MPCC reference implementation separates the obstacle-side/global
candidate search from the QP refinement: a one-dimensional DP path is
converted into a corridor before MPCC solves it. This supports keeping static
geometry preparation outside repeated numerical refinement while retaining
the final nonlinear certificate. It does not justify changing homotopy,
clearance or production authority in this Slice.
