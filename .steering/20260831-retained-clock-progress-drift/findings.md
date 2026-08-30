# Findings

## Observed symptom

Artifact sequence 931 remained the last accepted normal authority. It was
retained while the vehicle fell behind its planned schedule, exhausted its
cursor at decision 1802, and was followed by Emergency Stop and wall-margin
Recovery.

## Earliest actionable failure

The earlier immutable snapshot at sequence 992 / decision 1726 still had a
certified normal continuation. Persistent A and stateless direct B failed, but
a physical diagonal C from stage 5 to the finite target-tube boundary at stage
14 passed the unchanged seven-state SQP, nonlinear wall proof, timed opponent
proof and terminal Stop proof on the right side.

The left sibling was correctly rejected by exact dynamic proof. This is not a
clearance or tolerance relaxation.

## Root cause

The bounded current-world population did not represent the target encounter's
temporal topology. Its third member was generated only for a target valid over
the complete control horizon. When the canonical target tube ended at stage
13, the useful transition-to-boundary homotopy was omitted.

Two structural implementation defects amplified the omission:

1. a stateless seed copied forced candidate-schedule fields from the captured
   failed candidate, so a `DirectSide` seed could still execute the old
   diagonal/disjunction topology;
2. candidate construction held a reference into `std::vector` and then
   appended another candidate, so reallocation could invalidate the reference
   used to derive the third candidate.

## Causal chain

```text
captured failed candidate schedule leaks into stateless seed
  + candidate vector reference becomes invalid after append
  + finite target-tube boundary has no explicit candidate
  -> no certified fresh replacement after sequence 931
  -> retained sequence 931 remains sole normal authority
  -> cursor expires after the vehicle has fallen behind
  -> Emergency Stop near wall
  -> actual footprint wall-margin Recovery
```

## Repair

- reset all candidate-specific forced schedule fields when constructing the
  stateless current-world seed;
- derive the complete bounded population from stable local objects before any
  vector move/append;
- when the canonical target tube has a finite contiguous validity interval,
  add a physical diagonal from the last nominal stay-behind stage to its first
  invalid stage;
- preserve the old late exact-disjunction when the target occupies the complete
  horizon;
- keep the population bounded to three and retain every existing proof.

## Evidence

On the frozen sequence-992 snapshot, production-right now reports:

```text
stage=accepted
candidate_source=encounter-boundary-physical-diagonal
candidate_count=3
lattice_transition=5
lattice_ahead=14
bundle=1
```

Focused stateless-maneuver tests pass 27/27. Dynamic validation remains required
before claiming that the downstream cursor-expiry episode is eliminated.
