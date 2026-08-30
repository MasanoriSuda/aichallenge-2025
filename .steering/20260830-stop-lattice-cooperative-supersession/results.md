# Results

## Implemented lifecycle contract

`LatestOnlyWorker` now has an opt-in cancelable submission form.  Each
accepted submission advances a worker-local generation and supplies the
running job with an advisory `SupersessionToken`.  The existing
`submit_latest(Job)` call path wraps the new internal representation but does
not inspect the token, so its one-running/one-pending behavior is unchanged.

The Stop lattice observation uses the token at deterministic safe boundaries:

- before population traversal;
- before each candidate solve;
- after the solver returns;
- after exact trajectory adaptation, wall proof and dynamic proof.

A newer accepted submission produces an explicit `Reason::Superseded` result
with no certified plan.  A solver call is never interrupted and the private
solver context keeps a single owner.  The result remains observation-only and
is counted separately from solver, wall, dynamic and certificate rejection.

## Static evidence

- the first cancelable job observes a newer accepted generation and exits;
- the newest pending job then runs with a current token;
- the legacy replace-pending and exception-containment tests still pass;
- a Stop evaluation superseded immediately after its first solve reports one
  attempted candidate, no certified plan and the exact immutable source;
- a non-superseded Stop evaluation still builds the same certified plan;
- source authority audit reports no Store, adapter or publisher edge from the
  live Stop lattice comparison;
- production parameters, candidate set, solver settings and proof policy are
  unchanged.

## Verification

- `make autoware-build`: passed (25 packages);
- full `multi_purpose_mpc_ros` CTest: 59 / 59 passed;
- aggregate tests: 2276, 0 errors, 0 failures;
- single-authority source contract: 92 / 92 passed;
- `git diff --check`: passed.

The final `colcon test-result` scan emitted an unrelated stale-build warning
for a missing `build/joycon_contract_guard/package.xml`; the selected package
test result itself completed successfully with zero failures.

## Remaining dynamic gate

Exercise the same `make dev2` scenario after the static commit.  Acceptance
requires live `superseded` observations when a difficult old epoch overlaps a
new submission, no all-68 obsolete completion ahead of that newest epoch, and
lower maximum result age than `output/20260830-224528`.  Production behavior
must remain unchanged because this Slice still has `authority=shadow,
selected=0`.

## First live result and falsification

The first dynamic run is under:

`output/20260830-230003/d1/autoware.log`

Cooperative supersession worked while newer Stop sources continued to arrive.
Observed superseded evaluations stopped after 3--8 candidates with result ages
of approximately 0.3--0.7 seconds.  Subsequent newest epochs then ran and
produced accepted exact certified observations.  This validates the worker
generation token and safe-boundary evaluator checks.

The original dynamic gate did not pass.  Episode 1 left ShiftOut through
Dynamic Mission wait, after which no further Stop source was submitted.  The
last ShiftOut observation exhausted all 68 candidates, computed for 6633.485
ms and was consumed 6.7800 seconds old.  Later controller evidence showed
certified Track/Cruise/Follow artifacts replacing Overtake authority, but that
replacement did not invalidate the dedicated Stop worker.

The refined root cause is therefore not a solver tolerance or candidate-set
defect.  Newer Stop submission handles supersession correctly; certified exit
from the eligible ShiftOut/Pass intent set lacks an explicit worker lifecycle
edge.  Static and dynamic acceptance remain open until that proven replacement
invalidates both running and pending observation work.

## Refined lifecycle implementation

`LatestOnlyWorker::invalidate_pending_and_running()` now advances the active
generation only when a current running generation still needs invalidation,
and discards a pending job when present.  Repeated invalidation is idempotent
for an already superseded running generation.  Bounded worker telemetry records
running invalidations and discarded pending jobs separately.

The normal evaluation calls this API only after the certified Store exposes an
exact identity match for a newly solved artifact whose intent is not ShiftOut
or Pass.  Missing solves, missing candidates and identity mismatches preserve
the prior observation generation.  This creates the previously absent source
lifecycle edge without relying on a Mission phase timeout.

Static verification after the refinement:

- `make autoware-build`: passed (25 packages);
- full `multi_purpose_mpc_ros` CTest: 59 / 59 passed;
- aggregate tests: 2278, 0 errors, 0 failures;
- single-authority source contract: 92 / 92 passed;
- running invalidation without replacement: passed;
- pending discard without execution: passed;
- `git diff --check`: passed.

## Second live result: candidate ownership falsified

The refined run is under:

`output/20260830-231135/d1/autoware.log`

Newer Stop submissions still superseded older running work, and several
Overtake episodes completed through Pass, Return and Idle.  However worker
telemetry remained `invalidated_running=0/discarded_pending=0`.  A Pass source
(`sequence=4915`, `decision=5790`) exhausted all 68 candidates in 2891.212 ms
and was consumed 3.610 seconds old.

The causal ordering rejects the candidate-owned invalidation design:

1. `Pass -> Return` occurred;
2. a canonical Return command crossed the publisher;
3. `Return -> Idle` and later Cruise publication followed;
4. an asynchronously completed Pass candidate then occupied the candidate
   Store and started/retained the Stop observation;
5. no candidate-based non-Overtake invalidation could retire it.

The Store contract already states that certification creates a candidate only
and that execution identity changes exclusively at the publisher boundary.
The remaining implementation therefore moves Stop observation submission and
invalidation to that boundary.  The dynamic gate remains open until a new run
shows publisher-driven invalidation and no final all-68 Pass tail after Return.

## Publisher-owned implementation

The exact solver `Snapshot` is now immutable provenance on its
`CertifiedPlan`.  Its identity must match the execution artifact, but its
presence grants no Store, command or publisher authority.  The selected plan
passes that provenance through `CanonicalNormalPendingActuation`.

Only after serialized actuation has crossed the ROS publisher and
`mark_executed()` or `record_published_bundle_source()` accepts the exact plan
does the controller submit a Stop observation for ShiftOut/Pass.  Repeated
publication of the same exact artifact is deduplicated.  Published
Track/Cruise/Follow/Return/Rejoin, external Stop, and publication override
invalidate the observation worker.  Candidate completion no longer has any
Stop worker edge.

Static verification of this ownership correction:

- `make autoware-build`: passed (25 packages);
- full `multi_purpose_mpc_ros` CTest: 59 / 59 passed;
- aggregate tests: 2281, 0 errors, 0 failures;
- single-authority source contract: 92 / 92 passed;
- matching source provenance retained and mismatched identity rejected;
- candidate worker contains neither Stop submission nor candidate-owned
  invalidation;
- `git diff --check`: passed.

The unrelated stale-build warning for
`build/joycon_contract_guard/package.xml` remains outside this Slice.

## Final dynamic acceptance

The publisher-owned lifecycle run is under:

`output/20260830-233212/d1/autoware.log`

The first complete episode established the required causal ordering:

1. `ShiftOut -> Pass` at `1788100371.889`;
2. a published Pass/ShiftOut source started the third Stop observation;
3. `Pass -> Return` at `1788100373.444`;
4. publisher-owned invalidation retired the running observation;
5. the result was consumed as `reason=superseded` after 24 candidates;
6. later Return, Idle and Cruise publication did not restart that observation.

Across the run, no published-source identity rejection and no all-68 Stop
observation occurred.  The maximum observed result age was 2.445 seconds and
the maximum candidate count was 24, compared with 6.780 seconds and 68
candidates in `output/20260830-230003`.  Worker telemetry remained idle after
each Return/normal publication until another independently published Overtake
artifact began a new episode.  Subsequent episodes also completed
`ShiftOut -> Pass -> Return`, including one normal `Return -> Idle` handoff.

Production authority was unchanged throughout (`authority=shadow,
selected=0`).  The dynamic evidence therefore accepts the publisher boundary
as the Stop observation lifecycle owner and closes this Slice.  The episode-1
external-Recovery handoff and separate wall-margin warnings remain production
behavior outside this observation-only lifecycle correction; they must not be
hidden by further Stop-worker changes.
