# Design: delete the residual DynamicEscape execution activation

## Alternatives

1. Special-case Track/Cruise eligibility while leaving DynamicEscape progress
   activation active: rejected; two incompatible execution owners remain.
2. Fabricate an execution context so the active progress formulation can run:
   rejected; this recreates the pre-Mission ShiftOut producer just removed.
3. Retain the last Cruise artifact across the producer gap: rejected; this is
   a grace/fallback and hides the ownership defect.
4. Restrict progress execution activation to canonical Overtake execution,
   and let DynamicEscape use normal Cruise scope: selected.

## Ownership after the change

- `Action::DynamicEscape` is tactical provenance and contributes the validated
  lateral contract, obstacle identity, preferred side and current target tube.
- Canonical intent is Track before race start and Cruise during the race.
- The normal Track/Cruise population assembles progress metadata, both
  homotopies, seven-state SQP, exact wall/opponent proof, terminal successor,
  certified Store and atomic publication.
- Only coherent canonical ShiftOut/Pass/Return identity activates Overtake
  execution scope.

## Atomic deletion

Remove `ActivationSource::DynamicObstacleEscape`, the corresponding request
field and resolver branch. `resolve_activation()` becomes an explicit
canonical-Overtake-execution resolver rather than a second tactical state
machine. Remove the stored activation source if it has no downstream semantic
consumer, and replace the old activation unit test with a regression proving
that tactical DynamicEscape cannot request execution scope.

The DynamicEscape stage-wise lateral bounds and current target tube are already
assembled independently of execution identity. They remain available after
normal Track/Cruise eligibility is restored; the selected normal artifact is
still rejected unless all unchanged physical and successor proofs pass.

## Falsifiers

- a pre-Mission DynamicEscape again disables Track/Cruise eligibility;
- DynamicEscape gains an Overtake execution identity or bypasses Gate A;
- its normal seven-state request lacks the dynamic obstacle tube;
- real ShiftOut/Pass/Return no longer activates execution scope;
- the missing-scope Emergency is merely renamed rather than removed;
- an unproved normal candidate gains publication authority.
