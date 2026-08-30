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
