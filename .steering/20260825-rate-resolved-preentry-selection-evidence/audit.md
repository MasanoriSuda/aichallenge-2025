# Audit

## Finding

The tactical five-state owner no longer publishes normal actuation, but it still
owns three Gate-A responsibilities:

- left/right objective comparison;
- immutable pre-entry artifact identity and lifetime;
- exact path evidence copied into the selected Mission.

The six-state shadow already proves solver feasibility, the exact swept wall
path and current target tube. It currently discards its objective and immutable
CertifiedPlan, so it cannot yet replace those Gate-A responsibilities.

## Decision

Do not tune parameters and do not promote authority in this Slice. Preserve the
missing six-state evidence and compare decisions first. This is the minimum
change that reduces uncertainty without adding another production owner.

## Root cause found during integration

The worker correctly produced a six-state selection, but the manual
worker-to-live `V2XBehaviorOutput` copy omitted that new field. Complete
CertifiedPlans were therefore visible on each side while the live comparison
remained default-invalid. This was an async DTO boundary defect, not a solver,
wall-margin or branch-ranking defect. The import now copies the selection and a
source contract protects that boundary.

## Dynamic finding

In `output/20260825-192536`, domain 2 emitted eight throttled comparison
snapshots:

- three snapshots contained a complete immutable six-state CertifiedPlan and a
  valid six-state selection;
- five snapshots failed closed at the six-state solver after maximum
  iterations;
- one valid selection agreed with the current five-state selection;
- in two snapshots six-state selected a physically proved side while five-state
  selected no side.

All observed prospective intents were `ShiftOut`. This establishes that the
six-state selection evidence crosses the async boundary and that the old Gate A
can suppress a branch which the new formulation proves executable. It does not
yet establish Pass coverage or current-world adoption safety, so production
authority remains unchanged.
