# Design: remove the pre-Mission ShiftOut producer

## Alternatives

1. Retain Cruise across the intent transition for longer: rejected; this adds
   a grace rule and preserves the false ShiftOut semantic.
2. Run Gate A for DynamicEscape before a Mission exists: rejected; it invents
   a second Mission admission lifecycle and duplicates the normal obstacle
   population.
3. Keep both producers and prefer whichever solves first: rejected; authority
   becomes scheduling-dependent and the split ownership remains.
4. Resolve DynamicEscape as normal Track/Cruise obstacle avoidance and delete
   its fabricated ShiftOut execution identity: selected.

## Ownership after the change

- `Action::DynamicEscape` remains tactical provenance: it records that the
  front obstacle activated lateral avoidance and which path source motivated
  the request.
- Its canonical control intent is Track before the race session and Cruise
  during the race session. Follow is not selected because the escape action
  explicitly suppresses the Follow cap while clearing the obstacle.
- The existing normal current-world population evaluates both physical
  homotopies with the seven-state SQP, exact wall proof, exact opponent proof
  and terminal successor proof.
- `Action::ShiftOut` remains canonical ShiftOut only after OvertakeLine/Gate A
  creates the real Mission identity.

## Atomic deletion

Remove DynamicObstacleEscape from `CanonicalExecutionIdentitySource`, its
request fields, resolver branch and promotion tests. No dormant alternate
producer remains. The normal problem keeps dynamic obstacle identity and tube
provenance independently of Mission identity.

The normal homotopy owner must synchronize on `dynamic_obstacle_id` while the
dynamic obstacle contract is active. Cruise intentionally has no Mission
target, so using `target_id` would otherwise preserve a side selection across
unrelated obstacle encounters.

## Proof chain

The selected change does not publish GapPlanner geometry. GapPlanner and the
DynamicEscape tracker remain inputs to the tactical snapshot. The only normal
publication path is still:

current world -> bounded normal homotopy population -> seven-state solve ->
physical wall proof -> dynamic obstacle proof -> terminal successor proof ->
certified Store -> atomic publication.

## Falsifiers

- a pre-Mission escape still requests canonical ShiftOut;
- a normal avoidance artifact lacks dynamic-obstacle proof;
- an actual ShiftOut starts without Gate A/Mission identity;
- changing obstacle target does not reset normal homotopy ownership;
- moving Emergency is merely renamed while the same intent mismatch remains;
- the change increases callback overruns enough to make live authority stale.
