# Design

## Hypotheses

| Hypothesis | Supporting evidence | Falsifier | Confidence |
|---|---|---|---|
| Candidate ordering alone bounds freshness | common legacy rank 42 moved to anytime rank 1 | all-68 epochs still produce multi-second age | rejected |
| The candidate set is physically infeasible | six epochs exhausted the set | 29/35 live epochs produced exact certified Stop plans | low |
| An obsolete running epoch blocks the newest world | worker is one-running/one-pending and cannot signal running work | a newer epoch starts before the old evaluation completes | high |
| Cooperative supersession can bound the obsolete tail | each individual selected solve is normally tens of milliseconds | result age remains multi-second after old epochs exit between solves | high |

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
It is not driven by a phase timeout.  The normal evaluation may invalidate the
Stop generation only when all of the following hold:

- the new normal solve has an execution artifact and certified plan;
- the certified Store exposes that exact immutable identity as its candidate;
- its intent is neither ShiftOut nor Pass.

A solver failure, identity mismatch or temporarily missing candidate does not
invalidate the previous generation.  This keeps lifecycle invalidation tied
to a proven authority replacement rather than an incidental calculation
failure.
