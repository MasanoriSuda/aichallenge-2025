# Design

## 1. Separate physical and nominal contracts

Keep two meanings explicit:

- **physical corridor**: unchanged wall/target bounds used by current-world
  proof and exact footprint certification;
- **nominal execution tube**: the physical interval contracted on both sides
  by the existing configured tracking reserve.

The QP solves the nominal tube for controllable future states `1..N`. State
zero is fixed by the observed initial condition, so it is checked only against
the physical corridor. The canonical plan stores the original physical
corridor plus the scalar reserve, and plan validation applies the same split.

## 2. Scope

Activate contraction from the sealed canonical intent whenever it requires an
execution side (`ShiftOut`, `Pass`, or `Return`). This includes pre-entry
left/right worker candidates before the OvertakeLine FSM becomes active.
Other canonical intents retain zero required reserve and their existing bounds.

## 3. Fail closed before publication

If any future physical interval cannot contain the requested reserve on both sides,
extended problem construction fails with an explicit stage/reason.  It does
not reduce the reserve, expand the corridor, or introduce a fallback.

## 4. Immutable identity

Use a distinct Overtake bounds schema ID.  The plan carries
`required_lateral_tracking_reserve_m`; validation rejects malformed reserve,
insufficient-width intervals and nominal states outside the contracted tube.
Both QP construction and schema identity derive from the same canonical
intent, rather than from mutable FSM activity.

## 5. Deleted ambiguity

The existing wall-aware reference remains a cost-shaping mechanism only.  It
is no longer the sole representation of Overtake tracking reserve.  No legacy
command path is added or retained by this Slice.
