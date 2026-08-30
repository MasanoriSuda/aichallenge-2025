# Design

## Hypotheses

| Hypothesis | Supporting evidence | Falsifier | Confidence |
|---|---|---|---|
| Candidate ordering alone bounds freshness | common legacy rank 42 moved to anytime rank 1 | all-68 epochs still produce multi-second age | rejected |
| The candidate set is physically infeasible | six epochs exhausted the set | 29/35 live epochs produced exact certified Stop plans | low |
| An obsolete running epoch blocks the newest world | worker is one-running/one-pending and cannot signal running work | a newer epoch starts before the old evaluation completes | high |
| Cooperative supersession can bound the obsolete tail | each individual selected solve is normally tens of milliseconds | result age remains multi-second after old epochs exit between solves | partially accepted |
| Certified candidate replacement is the Stop observation lifecycle owner | candidate identity can be joined to the solver result | a later candidate completes after a different intent already crossed the publisher | rejected |
| The exact published artifact must own Stop observation lifetime | Store explicitly separates candidate certification from executed/publication evidence | a candidate-only lifecycle edge correctly retires the final Pass observation | high |

## Worker contract

Add an opt-in cancelable submission API to `LatestOnlyWorker`.  Each accepted
submission receives a monotonically increasing worker-local generation.  A
`SupersessionToken` compares its own generation with the latest accepted
generation.  Existing `submit_latest(Job)` wraps the new internal job form and
ignores the token, preserving all current call sites.

The token is advisory.  It never kills a thread, interrupts OSQP or changes a
completed result.  A newer pending submission is the only normal condition
that supersedes a running generation; worker shutdown also supersedes it so
destruction cannot wait for a complete obsolete lattice.

## Stop evaluator boundary

The Stop evaluator accepts an optional observation control predicate.  It
checks the predicate:

1. before candidate enumeration begins;
2. before each candidate solve;
3. immediately after a candidate solve, before exact proof.

When superseded it returns `Reason::Superseded` with the immutable source
identity and completed attempt count.  It does not publish a certified plan.
The result may enter the observation mailbox so telemetry can distinguish
supersession from solver, wall or certificate rejection.

## Invariants

- the running solver call always completes normally;
- the shared private solver context has only one owner;
- no candidate or proof is accepted after the token reports supersession;
- the newer pending epoch remains the only next job;
- normal production Store and publisher remain unreachable from this path.

## Alternatives rejected

- Wall-clock deadline: conflates machine load with world obsolescence and is
  explicitly frozen.
- Solver tolerance or iteration reduction: changes feasibility semantics.
- More worker threads: would race on solver context and increase CPU tail.
- Publishing an old certified Stop anyway: violates current-world freshness.
- Stop-specific atomic flag in the controller: duplicates the generic worker
  lifecycle and hides the actual ownership defect.

## Live falsification and refined lifecycle edge

The first live run proved that a newer Stop submission supersedes a running
evaluation, but also exposed a distinct terminal case: after ShiftOut changed
to Dynamic Mission wait, no further Stop submission existed.  The final old
ShiftOut epoch therefore exhausted all 68 candidates even though subsequent
normal production artifacts had Track/Cruise/Follow intent.

The worker consequently also needs an explicit source-invalidation operation.
It is not driven by a phase timeout.  The first implementation tied that edge
to exact candidate replacement.  The second live run falsified that owner:
candidate certification is asynchronous and may finish after a newer Return
or Cruise command already crossed the sole publisher.  A candidate is proof
that a plan could be used, not evidence that it was used.

## Publisher-owned observation lifecycle

The Stop observation source must cross the same publication boundary as the
normal command before it may start or remain active.  The immutable solver
source is therefore carried with its certified plan and then with
`CanonicalNormalPendingActuation`; it is not read back from the mutable
candidate slot.

After the serialized command is successfully published and the selected plan
is recorded as executed/current-world Bundle source:

- ShiftOut/Pass with an exact source/artifact identity starts one observation
  for that published artifact;
- publishing the same artifact again does not restart or supersede its own
  observation;
- any published non-Overtake normal authority invalidates running and pending
  Stop observation work;
- a publication override or external Stop invalidates it at the final
  publisher-authority ledger;
- solver completion without publication has no Stop observation edge.

This is an ownership correction, not a new lease, timeout, fallback or phase
rule.  The last actually published certified artifact is the only persistent
observation source.
