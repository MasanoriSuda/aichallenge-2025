# Validation

## Root-cause result

The rate-resolved request was a six-state/three-input QP, but its source
`MpccProblemContext` was sealed as `VelocityProgress5State`.  A second private
enum in the command candidate then asserted a six-state label which had not
come from the solver artifact.  The earliest violated invariant was therefore
the producer identity, not solver feasibility, wall clearance or a runtime
parameter.

The repair gives the six-state request its own sealed
`VelocitySteeringProgress6State` context and carries that formulation through
the execution artifact, physical identity, retained proof and command
candidate.  The command-only duplicate enum was removed.  A five-state or
unresolved artifact now fails closed before it can become a candidate.

## Static validation

- Failure-first source-contract test failed before the implementation because
  no independent six-state context existed.
- The corrected source-contract suite passed: 31/31.
- `make autoware-build` passed: 25 packages.
- `multi_purpose_mpc_ros` CTest passed: 49/49 test targets, 1,890 tests, zero
  failures.
- Added negative tests for a five-state identity at the rate-resolved solver
  boundary and certified-command boundary.
- `git diff --check` passed.

`colcon test-result --verbose` reported a stale unrelated
`build/joycon_contract_guard/package.xml` lookup after the test run, while its
own final summary remained 1,890 tests, zero errors and zero failures.  No
source or generated artifact was changed to mask that workspace residue.

## Dynamic validation

Command:

```text
make dev2
```

Artifact:

```text
output/20260825-081954
```

Observed facts:

- d1 and d2 emitted accepted candidates with
  `formulation:velocity-steering-progress-6state`.
- Every candidate telemetry line remained `authority=shadow, selected=0`.
- Rate-resolved artifact and mailbox invalid/identity mismatch counts remained
  zero in the inspected windows.
- d2 final rate-resolved solve window was 81/81 solved with 73 current-semantic
  physical accepts; the final candidate window was 73/81 available.
- d1 exercised Track/Cruise briefly, then spent the traffic interval in Follow;
  its last Track/Cruise window solved 80/80 with 78 current-semantic physical
  accepts and produced six-state candidates while eligible.
- Final callback windows reported zero overruns in both domains.

Startup odometry absence and shutdown-time odometry/RViz/orchestrator messages
were observed.  They are outside this identity Slice and were not suppressed by
new fallback or timing logic.

## Decision

The six-state canonical identity Slice is accepted.  Production promotion is
not accepted yet: retained command availability is not continuous, and a
normal publisher must not fall back across formulations.  The next root-cause
Slice must classify and close those admission holes before atomically
connecting the six-state Track/Cruise owner and deleting the five-state owner.
