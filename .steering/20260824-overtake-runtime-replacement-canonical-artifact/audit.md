# Audit

## Observed phenomenon

The first Overtake entry is healthy, but a later DynamicWait alternate-side selection changes the
Mission before canonical authority exists for the replacement.

```text
611 PassPlan frozen generation=1 side=-1
613 entry commit accepted
621 first ShiftOut output = canonical-shiftout-retained

653 PassPlan frozen generation=2 side=+1
654 FollowPrepare -> ShiftOut (dynamic wait selected alternate Mission)
662 first replacement output = Emergency / async-pending
666 later output = canonical-shiftout-retained
```

## Data/control flow

```text
left/right extended five-state solve
  -> selected Mission + selected immutable plan
  -> V2XBehaviorOutput keeps selected plan in an entry-named slot
  -> runtime authority copies only *_replan_mission
  -> replace_frozen... rebuilds/finalizes Mission
  -> generation/side/phase changes
  -> canonical worker is now requested
  -> first cycle has no canonical plan
  -> Emergency async-pending
```

## Root cause classification

- Root: lossy runtime tactical artifact boundary.
- Contributor: replacement generation is assigned by the consumer rather than sealed by the
  producer artifact.
- Symptom: first-cycle Emergency and braking.
- Later evidence: alternating progress/corridor rejections after the plant state diverges.
- Mask: retained plans and Recovery eventually resume or terminate execution.

## Confidence

High. The log sequence is deterministic and the runtime replacement function has no canonical-plan
input or store operation, while the selected dual branch already produces that plan.

## Dynamic regression found during implementation

The first implementation Gate, `output/20260824-113223/d1/autoware.log`, exposed a more upstream
identity defect before any runtime replacement could be assessed:

```text
650 PassPlan frozen generation=1
651 preentry canonical rejected: plan generation=2,
    admission=intent-generation-mismatch
```

The asynchronous worker receives live generation 0, phase Idle and side 0. Its private tactical
snapshot then runs `evaluate_v2x_behavior()`, which is allowed to mutate its private
`OvertakeLineState` while constructing geometry. The dual five-state producer subsequently derived
the executable artifact generation from that mutated private state, producing generation 2 for a
live generation-0 request. The live consumer correctly rejected it after freezing generation 1.

This is the same authority-provenance class as the Mission-only runtime boundary: an executable
identity was re-derived after the immutable async request boundary. The correction therefore seals
generation, phase, FollowPrepare origin and side in `OvertakeArtifactIdentitySeed` before worker
evaluation. Both left/right solve and selection consume that seed; mutable worker state remains
geometry scratch state only.

The final static audit also found that the first replacement implementation prepared the new
canonical context before replacing the plan store. A context reset could therefore clear the old
plan and then reject the new plan, even though Mission rollback reported that the old Mission was
retained. `adopt_overtake_canonical_plan_context()` now resolves context read-only, performs the
all-or-nothing plan-store replacement first, and commits/reset the lifecycle only after acceptance.
This removes the partial rollback state rather than adding another runtime fallback.

## Post-fix evidence

`output/20260824-114633/d1/autoware.log`:

```text
633 PassPlan frozen generation=1 side=1
634 Idle -> ShiftOut
643 generation=1, phase=ShiftOut,
    solver=canonical-shiftout-retained,
    formulation=velocity-progress-5state,
    canonical=satisfied
```

There is no `intent-generation-mismatch`, repeated speculative freeze, or first-entry Overtake
`async-pending` cycle. This run did not produce a complete same/cross runtime replacement artifact;
the first Mission later reached DynamicWait because no current-side prefix was available and moved
to Recovery. A second entry later lost current-world retained proof. Those are separate
current-world/cursor defects and were not hidden with a grace, retry, legacy controller or parameter
change in this Slice.
