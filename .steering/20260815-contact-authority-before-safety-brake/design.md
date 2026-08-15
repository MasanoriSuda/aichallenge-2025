# Design

## Remove the authority cycle

The existing committed-corridor suppression recognizes recoverable contact as
acceptable geometry, but execution authority is limited to active
ShiftOut/Pass or an already published DynamicMissionWait prefix. In
FollowPrepare this creates a cycle:

1. Behavior needs front-danger suppression to remain Overtake.
2. The DynamicMissionWait executor needs Overtake to publish its prefix.
3. Front-danger suppression waits for that published prefix.

Add a narrow `recoverable_contact_path_acceptable` source. It is valid only
when the Pass contact context and the independently bounded contact classifier
are both active. This source may satisfy execution/prediction authority for
front-danger suppression; it does not alter target continuity, fixed-corridor,
inter-vehicle or identity gates.

## Separate target identity from speed ownership

Replace the misleading local `nearest_front_matches_locked_target` value, which
currently contains longitudinal speed ownership, with two explicit hazard
identity decisions:

- whether the current nearest-front ID equals the frozen Mission target; and
- independently, whether the active front-hazard hold target ID equals it.

This prevents ContactContinuation for one vehicle from suppressing a different
vehicle's emergency. Current front risk is masked only by the first decision;
held-hazard release/refresh is controlled only by the second. One match never
authorizes the other.

## Existing fail-closed path

Once the same-target danger is suppressed, the existing logic must still pass:

- recoverable side-contact classifier;
- current wall/contact margin checks;
- DynamicMissionWait hard-fault check;
- wall/lateral-acceleration horizon evaluation for the 0.15 m separation bias;
- contact duration, progress, heading and lateral-velocity limits.

No threshold is changed. If the prefix is wall-infeasible, it is not published
and existing Recovery/SafetyBrake handling remains available.

## Expected logs

For the reproduced rear-side contact:

- `ContactContinuation entered`;
- `danger_suppress=1` and Behavior remains/returns Overtake;
- `dynamic Mission forward prefix active ... contact=1`;
- no speed collapse caused solely by the same locked target's hazard hold.
