# Results: Return transition starvation audit

## Frozen failure chain

`output/20260831-134900/d1` requested Return Gate A every cycle from decision
1535 through 1563, but no Return result reached the consumer.  The retained
Pass then lost terminal Stop viability at decision 1563.

## Instrumented reproduction

`output/20260831-145722/d1` reproduced the same class of failure:

- ShiftOut entered Pass and latched front-overlap exclusion.
- At the first logged Return deferral the shared transition worker reported
  `submitted=127`, `replaced=101`, `started=25`, `completed=24`, with one job
  running and one pending.
- Return drafts from decisions 1523--1536 were submitted while no proposal was
  available.
- The running job was an older `MissionGateA` job from decision 1491.  It took
  `2824.032 ms` and completed only after Pass had already entered
  FollowPrepare and the vehicle was approaching the wall.
- Only then did the worker start the latest pending Return job, decision 1536.
  That Return job took `445.695 ms`, was published by the mailbox, but its
  exact physical proof was `stage-wall-rejected`.
- Before that completion, Pass changed to FollowPrepare and then Recovery due
  to `actual footprint intersects static wall`.

## Root cause

`MissionGateA`, `PassGateA` and `ReturnGateA` share one non-cancelable
`LatestOnlyWorker`.  New submissions replace only the pending job; they do not
interrupt the running job.  A 2.824-second active-same-side Mission solve
therefore caused head-of-line blocking of every Return request.  Rebuilding
the Return draft each control cycle retained only the latest pending draft,
but could not give it execution time.

The late Return result also exposed a second, downstream fact: decision 1536
was already physically wall-infeasible.  This does not refute the scheduling
defect because the first Return request at decision 1523 was never evaluated.
The next structural experiment must isolate Return execution, then observe
whether the earliest Return candidate certifies.  Do not add a wait timeout or
retain Pass longer.

## Verification

- `make autoware-build`: 25 packages passed.
- Package CTest: 59/59 passed.
- Bounded runs `output/20260831-145136` and `output/20260831-145345` did not
  reach Return; no conclusion was drawn from them.
- Bounded run `output/20260831-145722` reproduced and classified the defect.

