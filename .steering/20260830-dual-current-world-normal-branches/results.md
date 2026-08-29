# Results

## Outcome

The producer-side lifecycle defect is closed without changing production
authority, clearance, solver settings or vehicle limits. Cruise/Follow normal
avoidance now evaluates both physical homotopies from one immutable source,
publishes them atomically into a data-only branch bank and sends only the
preferred certified plan to the existing Store. If every ordinary source later
fails exact current-world proof, an untried same-epoch branch can enter the same
unchanged production proof chain.

Dynamic generation and invalidation were observed. A naturally occurring
alternate-branch adoption was not observed in this run, so that acceptance item
remains open rather than being inferred from static tests.

## Observed phenomenon

The frozen decision 3149 snapshot from `output/20260830-043824` had one normal
Follow source and two physical homotopies. The persistent selected plan and the
stateless left plan failed, while the stateless right plan passed the unchanged
seven-state SQP, exact wall/dynamic proof and exact terminal Stop certificate.
The live producer nevertheless retained only the preferred-first result, so the
publisher could not inspect the viable opposite branch after later
revalidation failure.

## Causal chain

1. One immutable normal-avoidance source produced negative and positive
   candidate requests.
2. Candidate order was biased toward the prior homotopy owner.
3. The population loop returned immediately after the first certified result.
4. The opposite result never crossed the asynchronous producer boundary.
5. The selected artifact later failed exact current-world revalidation.
6. Candidate, published and executed evidence were exhausted, leaving no
   certified normal source.
7. External Emergency correctly became the only authority even though the
   discarded opposite homotopy was physically feasible in the frozen world.

The Emergency response was therefore a symptom. The first violated invariant
was producer completeness: a dual current-world population destroyed one
homotopy before current-world selection could use it.

## Root cause

The normal population mixed two responsibilities: generating a complete
homotopy set and selecting the preferred member. Its preferred-first early
return made candidate availability depend on iteration order and persistent
homotopy state. This was a Mission lifecycle/candidate retention defect, not a
clearance or solver-tolerance defect.

## Changes

- Added a data-only, mutex-protected normal branch bank.
- Required same source epoch, complete physical certificate, side-specific
  sealed fingerprint and monotonic atomic replacement.
- Made an accepted empty replacement invalidate every older branch.
- Evaluated negative and positive normal branches concurrently with independent
  persistent solver contexts inside the existing background worker.
- Joined both results before atomic publication and deleted the producer's
  preferred-first early return.
- Preserved the normal preferred branch as the only Store candidate.
- Added last-resort evaluation of untried bank branches after candidate,
  published Bundle and executed evidence fail.
- Kept the unchanged exact wall, timed-obstacle, actuation, recursive terminal
  successor and production authority proof chain.
- Updated the homotopy owner for a bank-selected alternate only after that
  alternate passed production proof.
- Added source sequence, attempted side, proof reason and selected side to the
  retained-decision diagnostics.
- Invalidated the bank when a newer source has no physical snapshot or no
  certified branch, preventing stale-world reuse.

No new lease, grace period, timeout, retry, resume rule, fallback, feature flag,
solver tolerance, weight or clearance adjustment was introduced.

## Deterministic evidence

The branch-bank test covers:

- atomic dual publication;
- same-epoch and side-specific identity;
- sealed fingerprint validation;
- monotonic replacement;
- invalid-plan rejection;
- accepted empty replacement and stale-source rejection.

The source-contract test verifies both branches are evaluated, their work is
joined before atomic replacement, the old loop-level early return is absent,
ordinary sources precede bank inspection, exact production revalidation is
required and a missing current physical snapshot invalidates the bank.

The frozen architecture comparison was rerun on the modified code:

| Path | Result | Solve time | Terminal progress | Terminal velocity | Lateral reserve |
|---|---:|---:|---:|---:|---:|
| A: persistent selected | rejected | 147.131 ms | - | - | - |
| B-left: stateless current world | rejected | 145.297 ms | - | - | - |
| B-right: stateless current world | accepted | 88.524 ms | 7.557 m | 1.858 m/s | 0.805 m |

This reproduces the exit classification `A fails, B succeeds`: persistent
Mission lifecycle/candidate retention defect.

## Verification

- `git diff --check`: passed.
- Focused branch-bank and source-contract tests: passed.
- Complete package CTest: 55/55 passed.
- `make autoware-build`: 25 packages succeeded.
- ROS topic, service, message, Domain and submission contracts: unchanged.

## Dynamic trial

Run: `output/20260830-052104`, `make dev2`, stopped after about two minutes.

- `selected/normal-avoidance`: 18 producer windows.
- `no certified normal avoidance`: 8 producer windows.
- Runtime logs include both-certified cases and cases where the preferred
  negative side failed while the positive side was certified and selected.
- `normal_branches=inspected:1`: 6 retained-evaluator windows.
- Alternate selected from the bank: 0.

At each observed consumer inspection a newer source epoch had already replaced
the bank with an empty pair. This is the required immutable latest-world
behavior; using an older certified pair would have violated the design. The run
therefore proves dual generation, atomic replacement and stale invalidation,
but not natural alternate adoption.

The run also reached `ShiftOut -> Pass` once and then entered Recovery because
of `actual footprint wall margin violated`. That is a committed Overtake wall
feasibility failure and is outside this Cruise/Follow normal-branch Slice.

## Runtime comparison with `.steering/ano`

Current run:

- control callback: 173 windows, mean of window averages 4.792 ms, maximum
  71.186 ms, 59 overrun cycles in 19 windows;
- background rate-resolved computation: 78 windows, mean of window averages
  36.684 ms, maximum 298.750 ms;
- background solve component: mean 7.182 ms, maximum 64.980 ms.

Upper-run log `.steering/ano`:

- GMPCC: `N=20`, `dt=0.12`, 2.4 s horizon;
- 4484 main-solve samples: mean 24.699 ms, p95 43.700 ms, p99 66.400 ms,
  maximum 326.500 ms;
- a child process owns asynchronous `r0`/`r1` tactical solves; sends are
  normally non-blocking and individual branches may fail while the other is
  still usable.

The current implementation protects the 40 Hz callback with a latest-only
background worker, but both normal homotopies still share that worker's
scheduling envelope. Its long compute tail is a remaining scheduling risk and
is not addressed by tolerance or horizon tuning in this Slice.

## Existing patches and deleted edge

The exact proof, Emergency supervisor, retained plan sources and homotopy owner
remain. They were not bypassed. The removed behavior is only the early return
which discarded the second current-world homotopy. No obsolete producer was
replaced by a parallel command authority.

## Remaining concerns

1. A natural live case of exact alternate-bank adoption remains unobserved.
2. Background dual-compute latency has a heavier tail than desired, although it
   does not run synchronously in the control callback.
3. The observed Pass wall-margin Recovery belongs to a separate committed
   Overtake feasibility family.
4. The short run did not produce result JSON and is not a six-lap performance
   acceptance test.

## Next dynamic checks

- Confirm a trace where the ordinary source fails, `normal_branches` attempts
  the untried side and exact proof either accepts it or records a physical
  rejection.
- Confirm source sequence and problem fingerprint match the inspected bank
  epoch and that no mixed/stale pair is used.
- Confirm no Overtake no-return side reversal is introduced.
- Treat the Pass wall violation and background scheduling tail as separate
  audits; do not tune clearance or solver parameters from this result.
