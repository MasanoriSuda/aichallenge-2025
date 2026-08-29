# Design

## 1. Immutable terminal policy provenance

Extend `ReplayWorld` with the exact moving-Stop policy and physical braking
limit used by production. Include every field in snapshot serialization,
validation and the interaction fingerprint. New artifacts use a new schema;
old schema data never acquires a fabricated certificate.

## 2. Proof-boundary snapshot

Add an observation-only interaction recorder which persists the complete
`mpcc_rate_resolved_shadow::Snapshot` without requiring a rejected QP. The
record is keyed by intent, physical homotopy, proof stage and failure outcome.

When retained production reports `TerminalContingencyUnavailable`, construct a
counterfactual current-decision snapshot from the current semantic problem and
the predecessor which entered the callback. This snapshot has no Store,
mailbox, publisher or authority path. It exists only for offline comparison.

## 3. Exact Stop suffix certificate

After an arm produces a normal exact trajectory accepted by wall and dynamic
proof:

1. extract the arm's first executable actuation;
2. start from that arm's exact control-origin state;
3. replay that selected actuation for one publisher interval;
4. run the exact production path-feedback law under maximum braking;
5. reconstruct and sweep the resulting footprint against the same wall grid;
6. evaluate the same current-world dynamic obstacle set;
7. accept the `ManeuverBundle` only if both normal and Stop trajectories pass.

`ManeuverBundle` owns the exact Stop trajectory and its wall/dynamic
certificates, not only a declarative intent.

## 4. Compatibility

`RecordedInteractionSnapshot.recorded_qp` becomes optional. A/B/C/D/G use the
sealed semantic/world source. Exact-QP replay and external-primal arms reject a
source-only artifact explicitly. Existing QP snapshots retain their replay
capability.

## 5. Production boundary

No production admission rule changes in this Slice. Once the frozen failure is
classified, a later Slice may make a physically certified Stop suffix part of
fresh normal admission, or replace the candidate/path representation if an
alternative arm proves feasible.
