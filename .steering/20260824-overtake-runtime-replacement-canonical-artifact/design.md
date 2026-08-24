# Design

## Root cause

The runtime dual five-state MPCC producer already preserves an immutable canonical plan for the
selected branch. `V2XBehaviorOutput`, however, exposes runtime replacement as a Mission-only pair:

```text
mpcc_lite_*_replan_ready + mpcc_lite_*_replan_mission
```

`replace_frozen_overtake_mission_after_dynamic_replan()` rebuilds and freezes that Mission, updates
generation/side/phase, and does not receive or store the plan selected by the same solve. The live
controller must therefore wait for the separate rolling worker after authority has already moved.

The first `async-pending` Emergency is the direct symptom. Later progress/corridor failures are
partly amplified by the braking and state change, but must be reassessed after the authority gap is
removed.

## Selected design

Introduce a typed `OvertakeExecutionArtifact` with:

- selected `OvertakeMissionCandidate`;
- immutable `CanonicalExecutionPlan` from the same selected dual branch.

The producer seals the plan for the prospective Mission generation. The live runtime replacement
then performs one transaction inside a single control callback:

1. validate the Mission's geometric/dynamic replacement contract;
2. validate canonical plan completeness and identity against prospective generation/target/side;
3. freeze the candidate Mission into rollback-protected state;
4. resolve the prospective canonical lifecycle without mutating live context;
5. atomically replace the plan store, then commit the already-resolved lifecycle identity;
6. move the FSM phase without another fallible planning step.

If steps 1--5 fail, the existing Mission, phase and canonical plan remain authoritative. In
particular, context reset is no longer allowed to clear the previous plan before the replacement
plan has been accepted.

## Scope

This Slice covers the MPCC-lite same-side and cross-side replacement paths which currently own
production runtime tactical replacement. Legacy/opponent-side call sites are not granted canonical
authority and are not expanded with another compatibility artifact. Their reachability is recorded
for the later legacy deletion Slice.

## Rejected alternatives

### Add a one-cycle Emergency grace

Rejected: it hides the producer/consumer loss and leaves phase ahead of authority.

### Re-solve synchronously during replacement

Rejected: it duplicates the selected dual solve and puts solver latency in the 40 Hz callback.

### Keep Mission and plan in independent optionals

Rejected: independent lifetime/readiness permits partial states and repeats the current defect.

### Relax progress or corridor validation

Rejected: those failures are downstream evidence. Their validity can only be judged after atomic
artifact adoption.

## Logging

Runtime outcome logs must report artifact presence, admission reason, plan id, prospective
generation, target and side. A ready flag without a complete artifact is an explicit invariant
violation, not a silent fallback.
