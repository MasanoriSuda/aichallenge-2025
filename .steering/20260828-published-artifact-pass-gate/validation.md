# Validation

## Static gates

- `make autoware-build`: passed, 25 packages.
- complete `multi_purpose_mpc_ros` package test suite before the final log-only
  aggregation edit: 2,032 tests, zero failures.
- focused resolver test after the aggregation edit: 8/8 passed; aggregate
  result contained 1,981 tests, zero errors and zero failures.
- `git diff --check`: passed before the dynamic gate and is repeated before
  commit.

## Dynamic evidence

The first bounded run, `output/20260828-222217`, entered one ShiftOut episode.
The exact last-published artifact was aligned successfully 47 times.  The
frozen failure strings did not recur:

- `Pass entry physical wall gate unresolved`: 0;
- `dynamic Mission wait has no wall-feasible lateral authority`: 0;
- `actual footprint wall margin violated`: 0.

The episode did not reach the Pass boundary.  It remained in ShiftOut while
normal authority intermittently fell into Emergency, then terminated on the
pre-existing `same-target Mission total budget expired` condition at 15 s.
That outcome neither disproves the identity repair nor provides direct dynamic
acceptance of ShiftOut-to-Pass.  No timeout or clearance was changed to make
the run pass.

The initial decision log emitted once per newly published sequence (47 lines).
That was excessive.  Logging was reduced to activation/rejection-reason
changes; artifact sequence remains present in each state-transition record.

Two independent runs, `output/20260828-222947` and
`output/20260828-223138`, did not exercise this Slice.  Both lost Track/Cruise
normal authority before ShiftOut and repeatedly published
`canonical-normal-emergency-stop`.  The latter logged 30
`physical obstacle world does not match problem identity` assembly rejects.
Because the new alignment code is gated to ShiftOut or ShiftOut-origin
DynamicWait, these are separate upstream failures and are not attributed to
this change.

## Comparison checkpoint

The reference log `.steering/ano/autoware - 2026-08-21T211659.829.log` uses a
single always-running GMPCC controller (`N=20`, `dt=0.12 s`) and contains 4,466
normal `solved` summaries, including solve times above one control period.  It
continues to command from its usable receding solution rather than requiring a
new result to share the next 40 Hz world identity exactly.

The current blocker is therefore frozen separately as an asynchronous
world-identity/lifecycle problem, not handled by extending a Mission budget or
relaxing physical proof.

## Decision

The code Slice is accepted as the structural correction for split execution
identity: the Pass/DynamicWait supervisor can now consume only the exact
published ShiftOut artifact and advances it with the publication clock.  It
does not claim full dynamic Pass acceptance; that gate remains pending until a
run reaches the boundary without the separately typed normal-authority
failure.

The next Slice must audit immutable obstacle-world provenance versus live
current-world revalidation.  It must not weaken exact wall/opponent proof or
introduce a lease merely to make delayed worker results appear fresh.
