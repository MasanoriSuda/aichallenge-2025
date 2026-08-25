# Design

## Baseline observation

The first visible sustained QP failure in Domain 3 occurs at decision 2823,
but it is not the first defect.  At decisions around 1746 the controller starts
alternating:

```text
certified retained six-state command
-> retained-proof-unavailable emergency command
-> certified retained six-state command
-> retained-proof-unavailable emergency command
```

The telemetry window classifies the rejected joins as steering reachability,
velocity reachability and delay/control-path failures.  The emergency command
then becomes the next committed input, so a proof rejection mutates the state
against which the next asynchronous artifact is judged.  This creates a
closed-loop discontinuity before the later OSQP and Recovery symptoms.

## Audit questions

1. Does retained revalidation compare the artifact actuation at the control
   origin to a physical state projected to that same origin?
2. Is reachability measured over the actual artifact age/control-prefix time,
   or only over one publication interval?
3. Does the speed reachability check compare states at the same timestamp?
4. Can a fail-closed emergency command invalidate the next artifact solely
   because the proof clock and actuator clock differ?
5. Is the first wall/path rejection upstream of authority loss, or a result of
   the alternating emergency command?

## Repair rule

The repair must change the shared causal/time contract, not relax a safety
threshold.  The sealed retained proof must compare physical and planned state
at one explicit time origin.  A genuine unreachable state, blocked delay
prefix, changed world, stale observation or wall contact remains rejected.

The plan lifecycle must also distinguish three events that were previously
collapsed into one store replacement:

1. solver completion;
2. physical proof certification;
3. successful publication of the exact selected command.

Only event 3 creates retained execution evidence.  A newer certified candidate
may be attempted first, but it cannot overwrite the last command-producing
plan merely because an asynchronous worker finished.

## Selected structural repair

- `Store::candidate_snapshot()` exposes the newest physically certified
  solver candidate for current-world admission.
- `Store::snapshot()` exposes only the last plan whose exact canonical command
  crossed the publisher boundary.
- `Store::mark_executed()` is called only after the final command was published
  and matched the selected canonical command without mutation.
- Current-world admission tries the newest candidate, then the distinct last
  executed plan.  The latter is a same-formulation retained source, not a new
  fallback or authority.
- Fresh and retained physical checks share the exact current-to-control prefix;
  the steering initial state is the projected physical observation, while
  publication reachability is checked from the previous published command over
  one publication interval.

## Later Slices

After continuity is dynamically accepted:

- collect one decision-trace timeline for ShiftOut, direct Pass, Pass and
  Return;
- replace the direct-Pass five-state Gate A only after six-state dynamic
  evidence exists;
- delete the replaced five-state lifecycle and then the remaining Slice 6
  migration code;
- run the multi-domain integrated quality gate before parameter tuning.
