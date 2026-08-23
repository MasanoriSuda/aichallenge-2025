# Design

## Causal chain

1. Overtake five-state MPCC can produce a physically certified canonical plan.
2. That plan is not stored or selected by production Overtake authority.
3. A later fresh-solve miss therefore has no same-formulation continuation.
4. Production falls through conversion and three-state fallback.
5. This preserves the MPC/MPCC authority split that the migration is intended
   to remove.

The correction in this Slice is not another fallback. It establishes the
missing proof needed to continue the same five-state plan.

## Observation contract

`OvertakeDynamicCorridorObservation` describes the current target-dependent
feasible corridor at control-horizon timestamps. Its fingerprint seals:

- target identity and current V2X observation generation/time;
- whether target exclusion is physically encoded in the corridor, or whether
  the existing current/predicted footprint separation gate certified release;
- elapsed-time knots and lateral lower/upper bounds.

The corridor is the same combined wall/obstacle contract consumed to build the
current control problem. Exact wall geometry remains independently checked with
the occupancy grid and yawed vehicle footprint.

## Retained proof

For the exact remaining cursor window:

1. Lift current circular progress to the retained plan branch.
2. Rebuild the current course-frame window.
3. Check the measured-to-control delay prefix against the wall map.
4. Check the connector from current control pose to expected retained pose.
5. Interpolate the current dynamic corridor at time zero and every remaining
   retained stage endpoint.
6. Reject if current/expected lateral state or any remaining endpoint is outside
   the current corridor.
7. Check every remaining world-space segment against the wall map.
8. Seal a retained physical certificate using current decision provenance.
9. Extract actuation and prediction from the same immutable plan and cursor.

The plan's old obstacle generation is deliberately not reused as evidence. The
proof is stamped with the current target observation and current corridor
fingerprint. Intent, Mission generation, and target identity must still match.

## Plan-store rule

Only a fresh result whose command, plan, cursor, physical certificate, and
prediction are all complete may replace the store. Any partial or rejected
fresh chain leaves the previous accepted plan unchanged.

## Authority boundary

This Slice records whether fresh or retained canonical authority would be
available. It does not return that authority from `get_control()`. Promotion is
a later Slice and requires dynamic evidence for fresh misses; no legacy branch
is deleted here.
