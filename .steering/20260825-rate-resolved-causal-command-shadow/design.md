# Design

## Causal order

The required order is:

1. Build and evaluate the current five-state Track/Cruise candidate.
2. Resolve its production output (canonical command or EmergencyStop).
3. Let that resolution update `previous_steering`.
4. Bind the sealed six-state draft to that exact steering predecessor.
5. Submit the immutable asynchronous six-state snapshot.

This is the same causal rule already used by Follow: the next asynchronous
problem is born only after the current output becomes committed history.

## Submission draft

`evaluate_canonical_normal_shadow()` may create a
`RateResolvedTrackCruiseSubmissionDraft`, but it cannot submit it.  The draft
contains only immutable semantic inputs produced by the already-built extended
problem:

- rate-resolved adapter request,
- course-progress origin,
- horizon size,
- source problem context.

The outer control cycle retains the source `MpcProblem`, binds
`current_steering_rad` after output resolution, and constructs the existing
solver/physical snapshots without rebuilding the optimization problem.

## Command candidate

A separate `RateResolvedCommandCandidate` represents the first executable
actuation from an accepted retained proof.  It is deliberately not
`CanonicalNormalCommand`, whose current contract is five-state-specific.

The candidate carries:

- current decision ID,
- artifact sequence and source decision ID,
- source problem fingerprint and stage geometry ID,
- intent,
- control-stage index,
- predicted speed, acceleration, steering rate, steering angle, curvature, and
  virtual-progress speed.

The builder is pure and rejects missing/invalid identity or non-finite
actuation.  The controller stores no command ownership and only records deltas
against the authoritative five-state output.

## Rejected alternatives

- Rebuild the extended problem after output resolution: duplicates expensive
  work and can produce different semantics from the solved five-state problem.
- Clamp the first six-state steering after solve: mutates certified evidence and
  hides the causal bug.
- Promote retained six-state output immediately: command provenance and dynamic
  acceptance are not yet proven together.
- Reuse `CanonicalNormalCommand` while setting formulation to five-state: false
  provenance.

## Remaining production blockers

- Define the six-state production command/publisher contract and remove the
  five-state Track/Cruise owner atomically when promoted.
- Resolve the explicit time semantics of the measured-to-control connector
  versus the retained suffix origin.
- Obtain dynamic evidence for fresh and retained command candidates under
  Track and Cruise, including worker replacement and rejection cases.
