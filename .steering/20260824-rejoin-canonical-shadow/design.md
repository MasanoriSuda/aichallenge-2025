# Design

## Root cause

The orchestrator correctly maps line `Recovery` to `ControlIntent::Rejoin`. Downstream code then
excludes Rejoin from four independent domains: canonical normal artifacts, progress metadata,
five-state phase activation and the Overtake async producer. Consequently every Rejoin cycle
falls through to the old three-state normal owner.

The wall violation is the trigger for Recovery, not the cause of the authority switch.

## Rejected alternatives

### Add Rejoin to Overtake production predicates

Rejected for this Slice. Overtake retained proof requires target/corridor identity that is not the
semantic owner of a base-line Rejoin. Promotion without Rejoin-specific dynamic evidence would
only replace one hidden fallback with an unproven authority.

### Treat Rejoin as Track/Cruise

Rejected. Sharing a plan store or warm-start lifecycle across Track/Cruise and Rejoin permits a
different intent's artifact to become an accidental retained candidate at the transition.

### Fail closed immediately

Rejected as the final migration. It would expose the split authority but remove the only current
lateral path that moves the vehicle back from an Overtake wall-margin failure.

## Selected structure

1. A pure `resolve_rejoin_shadow_eligibility()` owns the observation boundary.
2. `init_problem()` requests progress metadata for Rejoin while leaving
   `progress_contouring_active=false`; production therefore remains unchanged.
3. The existing Track/Cruise fresh canonical evaluator becomes a shared normal-shadow evaluator
   parameterized by a typed mode.
4. Rejoin receives its own solver context, warm-start identity/epoch and plan store.
5. The Recovery line remains the lateral reference. Stage bounds remain the QP contract, and the
   fresh physical certificate checks the actual bare footprint from the current pose through the
   complete solved horizon.
6. Retained Rejoin selection is not promoted by this Slice. Shadow telemetry may expose that the
   generic retained proof is unsuitable; that result informs the next design Slice.

## Why canonical artifact support is expanded now

Shadow evidence must exercise the same immutable five-state artifact, cursor, actuation and
authority selector contracts used by production. Marking Rejoin as a supported artifact intent
does not grant runtime authority; publisher routing remains unchanged. Rejoin does not require a
target or pass side because its semantic goal is the base racing line.

## Expected production behavior

None. During Rejoin the existing legacy command remains published. In parallel, logs state whether
the five-state Rejoin candidate reached a complete canonical fresh chain and why it was rejected.
