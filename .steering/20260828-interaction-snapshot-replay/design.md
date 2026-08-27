# IM-1 immutable InteractionSnapshot design

## Data flow

```text
one current control callback
  -> canonical seven-state semantic request
  -> physical wall snapshot
  -> complete V2X current-world observation
  -> owned async Snapshot
  -> failure recorder
  -> snapshot.yaml + wall-grid.bin
  -> deterministic loader
  -> A / B / C / D replay consumers
```

The recorder remains failure-only and observation-only.  It does not schedule
another solve or return a candidate to production.

## Replay world

The async snapshot gains an owned `ReplayWorld` containing:

- observation generation and observation time;
- current-world completeness;
- current measured wall-monitor pose;
- exact measured-to-control pose prefix;
- static wall fingerprint and hard-clearance proof inputs;
- all valid V2X vehicles at the captured epoch, including position, velocity,
  acceleration, covariance and resolved physical radius.

The occupancy grid remains a `shared_ptr<const ...>` and is serialized to the
existing binary payload.  No live planner or subscriber object crosses the
worker boundary.

## Interaction seal

The interaction fingerprint covers semantic identity, control origin,
seven-state request, stage path geometry, wall proof inputs, grid fingerprint
and ordered obstacle observations.  Obstacles are sorted by ID before sealing
so container iteration order cannot change identity.

The loader recomputes the seal.  It does not trust a serialized boolean.

## Backward compatibility

The existing snapshot schema remains readable by `load_recorded_qp` and
`mpcc_qp_replay`.  Interaction replay requires the newly serialized world
payload and valid seal.  Older artifacts return an explicit incomplete reason
instead of silently inventing missing world state.

## Timing and authority

The only live cost is copying already-owned current-world data into a failure
candidate snapshot and hashing it.  Disk I/O remains bounded by the existing
failure-family deduplication.  The loaded type exposes data only; it has no
mailbox, publisher, certified-plan store or command conversion.

## `.steering/ano` comparison

After the native contract passes, IM-4 will map upper-run encounters into the
same schema where source data exists.  Fields absent from the upper logs remain
`Unknown`; they are never synthesized.  Cross-run comparison uses encounter
metrics and never shares causal fingerprints across different controllers.
