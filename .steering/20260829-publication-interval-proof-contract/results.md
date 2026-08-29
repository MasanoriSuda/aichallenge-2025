# Results

## Outcome

The publisher/solver proof-granularity mismatch is repaired and dynamically
observed. The target contract is accepted. The complete two-domain race gate is
still open because domain 1 exposed an independent steering/velocity join
failure after the new prefix had already been physically certified.

## Failure-first regression

`RetainsPublisherIntervalWhenLongSolverStageLeavesCorridorLater` was added
before the implementation. On baseline `8dc45378` it failed with
`ExactTrajectoryRejected`: the first solver stage was 100 ms, its first 25 ms
were clear, and the trajectory left the corridor later in the same stage.

After the correction:

- the same case returns `PublisherIntervalPrefix` ending at exactly 25 ms;
- a trajectory which leaves the corridor inside 25 ms remains rejected;
- a remaining artifact suffix shorter than 25 ms remains rejected because it
  cannot prove the command held on the wire.

## Static validation

- `make autoware-build`: passed.
- physical-adapter focused tests: 18/18 passed.
- retained-revalidation focused tests: 49/49 passed.
- command/production-adapter focused tests: 12/12 passed.
- complete `multi_purpose_mpc_ros` package test: 2,132 tests, 0 errors,
  0 failures, 0 skipped.
- `git diff --check`: passed before dynamic execution.

`colcon test-result` also reports a stale missing
`build/joycon_contract_guard/package.xml` result entry, but the selected
package itself completed with all 2,132 tests passing.

## Dynamic validation

Run: `output/20260829-175025`, `make dev2`.

### Domain 2: positive target evidence

Domain 2 obtained and published canonical seven-state normal authority while
moving at approximately 3.3--4.4 m/s. The retained validator repeatedly
observed:

- continuation exact reason `invalid-lateral-bounds` later in the suffix;
- continuation/static/dynamic scope `publisher-interval-prefix`;
- one proved control input;
- publisher-interval wall and dynamic proof clear;
- terminal Stop attempted and certified;
- final retained reason `accepted`.

Examples include decisions 1756, 2412, 2442, 3254, 3453, 3465, 3729 and 3749.
Aggregate telemetry windows reported publisher-interval acceptance counts of
14, 3, 17, 6 and 2 while retaining 81/81 accepted proofs in those windows.
This is the same causal class as the frozen decisions: a later suffix defect no
longer erases a clear next publication command.

### Domain 1: independent integration blocker

Domain 1 generated the new prefix and certified terminal Stop, but the final
retained result was often `steering-unreachable` (and later velocity join
loss) before production admission. It therefore remained near zero speed and
held the external Stop authority. This is not a continuation-prefix failure:
the log explicitly shows clear publisher interval, exact accepted prefix and
certified terminal Stop before the steering join rejects the transaction.

The next Slice must freeze the first domain-1 steering/velocity join failure
and trace its producer/consumer time origins. It must not modify the
publication proof, steering limit, rate limit, solver tolerance, clearance,
lease or fallback merely to make this run move.

## Contract changes

- Removed remaining-current-solver-stage partial authority semantics.
- Added one exact publisher-interval proof scope shared by nonlinear
  continuation, wall proof and dynamic-obstacle proof.
- Kept terminal Stop as a mandatory certified suffix for all partial normal
  authority.
- Added telemetry names which expose the actual proof unit.
- Added no new normal authority, timeout, grace, lease, fallback or parameter.

## Remaining risks

- Overall `make dev2` race quality did not pass because domain 1 remained in
  Stop after an independent command-state join failure.
- Near a solver-stage boundary with less than one publication interval
  remaining, the retained artifact intentionally fails closed. A fresh artifact
  is required; no implicit piecewise command is inferred.
- The dynamic run did not exercise ShiftOut/Pass because domain 1 did not become
  a normal moving overtake target. The repaired proof itself was exercised at
  speed in domain 2.
