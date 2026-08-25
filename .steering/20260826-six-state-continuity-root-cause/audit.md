# Root-cause audit

## Observed symptom

In Domain 3 of `output/20260826-050947`, normal Cruise authority first became
unavailable around decision 1746.  A retained six-state command then alternated
with the canonical emergency stop.  The resulting acceleration commands
alternated between approximately `+1.0 m/s2` and `-3.0 m/s2`, causing the
visible speed collapse.  Sustained OSQP maximum-iteration failures appeared
much later and are downstream symptoms.

## Failure classification

The original two-second aggregate contained several retained rejection kinds,
so it could not identify the first event.  A reason-transition trace was added
without changing authority or control behavior.  The bounded reproduction
`output/20260826-054917` identified the first moving Domain 3 rejection:

```text
decision=1754
previous=accepted
current=delay-prefix-blocked
steering=current:0.177467/expected:0.173715
velocity=current:6.477587/expected:6.482641
delay_path=valid:1/clear:0/reason:collision
reject_pose=(89648.442,43145.200,1.981)
```

The steering and velocity joins were valid at that decision.  The first cause
was the wall collision predicted on the measured-to-control-origin prefix.

## Causal chain

1. The canonical execution adapter projects raw odometry through the configured
   130 ms actuation delay using the measured yaw rate.
2. Retained current-world revalidation sweeps that exact curved prefix before
   accepting the sealed suffix.
3. Fresh production wall certification did not carry that prefix.  It swept a
   straight segment from the raw current pose to the first reconstructed MPCC
   horizon pose.
4. On a curve, the straight chord can be wall-clear while the physically
   executed delay prefix intersects the wall.
5. A plan could therefore enter the certified store under a weaker wall proof,
   then be rejected by the stronger runtime proof.
6. Loss of canonical normal authority produced the emergency command; the next
   asynchronous plan was evaluated against the changed vehicle state and the
   normal/emergency alternation amplified the deviation.

## Root cause

Fresh plan admission and retained execution used different geometric objects
for the same current-to-control interval.  This was a proof-contract mismatch,
not a wall-margin, OSQP, speed, acceleration or steering tuning problem.

## Failure-first regression

`RejectsCurvedControlPrefixHiddenByClearChord` constructs a wall cell that is
intersected only by a curved control prefix.  Before the repair the test failed
because the physical certificate returned `Accepted`; its straight chord did
not see the collision.

## Structural correction

- The immutable physical-wall Snapshot now contains the exact canonical
  control prefix.
- Snapshot identity fingerprints that prefix rather than only the raw current
  pose.
- Fresh wall certification validates the prefix separately, then sweeps from
  the exact control-origin endpoint through the MPCC horizon.
- Retained revalidation keeps its independent current-world prefix check.
- Rejection diagnostics expose the exact prefix/connector collision point and
  current-versus-expected actuation at reason transitions.

No fallback, lease, timeout, feature flag or parameter adjustment was added.

## Remaining proof obligation

A moving dev3 run must show that a plan whose exact prefix is blocked is never
published into the certified store.  Genuine prefix blockage must still fail
closed; acceptance followed by a same-cycle prefix contradiction must not
occur.

## Second incident after prefix repair

The prefix repair removed the first same-cycle wall-proof contradiction, but
`output/20260826-065438` exposed a second, independent continuity defect.  The
worker continued to certify a new plan nearly every cycle.  Runtime admission
then reported repeated `steering-unreachable` results, including cases where
the newest plan expected a steering command outside the single-publication
reachable interval from the actually published command.

The first interpretation was that the physical observation should be replaced
by the previously published command.  The failure-first steering tests rejected
that interpretation: the six-state initial steering remains the physical
observation projected to the control origin.  The previous published command
is only the predecessor for the next publication slew constraint.

## Deeper root cause

The certified-plan store treated solver certification as if it also proved
execution.  An asynchronous worker could therefore replace the retained source
with a plan that had never produced a command.  Runtime revalidation compared
that unrelated command sequence against the vehicle's real publication
history, correctly rejected it as unreachable, and canonical authority fell to
Emergency.  The symptom was in the steering contract, but the upstream defect
was the store lifecycle and execution provenance.

The complete causal chain is:

1. asynchronous solver completes and physical wall proof accepts;
2. old store immediately labels that plan as the retained plan;
3. no exact command from that plan has yet been published;
4. next callback evaluates its suffix against the real prior published command;
5. steering reachability rejects the unexecuted sequence;
6. Emergency changes the plant state and amplifies subsequent discontinuity.

## Structural correction of the second incident

- Certification now creates a candidate only.
- A candidate becomes the executed retained plan only after its exact canonical
  command is successfully published.
- A newer candidate never destroys the last executed plan.
- Current-world evaluation records candidate and executed sequences and reasons
  separately and identifies which source was selected.
- Normal command mutation, failed publication or promotion rejection cannot
  create retained evidence.
- Failure-first tests cover curved-prefix wall mismatch, pre-publication command
  causality, physical-versus-published steering semantics, certification without
  execution, and candidate replacement without executed-plan replacement.

Static verification completed with a successful full workspace build, 7/7
focused tests and 51/51 package tests.  Dynamic acceptance remains open.
