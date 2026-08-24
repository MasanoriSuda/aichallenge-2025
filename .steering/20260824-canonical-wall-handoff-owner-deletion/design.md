# Design

## Selected design

Define canonical Overtake ownership once at the node boundary from the typed
canonical command (`ShiftOut`, `Pass`, or `Return`). When present:

1. retire pending/retained legacy DynamicEscape execution artifacts;
2. reset solver, active-Overtake and DynamicEscape wall-handoff gates;
3. reset the DynamicEscape exit gate and its latched identity;
4. prohibit retained DynamicEscape restore and all legacy wall/exit monitor
   activation in that callback;
5. publish the exact canonical command unless the independent Emergency or
   stuck-Recovery supervisor overrides it.

The canonical solver and current-world proof keep wall and obstacle safety
ownership. This Slice removes only downstream duplicate normal authority.

## Rejected alternatives

### Increase DP source age or retained progress tolerance

Rejected. It masks the plant/plan divergence after an unauthorized brake and
weakens retained-plan identity.

### Allow both wall monitors and prefer the least restrictive result

Rejected. Two physical interpretations and two command owners preserve the
same nondeterministic handoff defect.

### Convert the old wall gate into another canonical fallback

Rejected. A stale outgoing prediction is not a certified candidate for the
current intent and must not be relabelled as one.

### Remove all emergency/recovery overrides

Rejected. Those are external supervisors, not competing normal controllers.

## Scope

This Slice applies to canonical Overtake normal commands only. Rejoin remains
outside canonical production scope and keeps its existing legacy handling
until its own migration Slice.

## Observability

Emit one state-change trace when legacy handoff state is retired, including
which gates/artifacts were present. The final decision trace remains the proof
that no legacy owner mutated the command.
