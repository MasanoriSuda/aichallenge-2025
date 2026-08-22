# Slice 1 validation

## Static evidence

- Test-first check: the initial build failed because
  `mpcc_execution_contract.hpp` did not exist yet. Production implementation
  was added only after this expected failure.
- `git diff --check`: passed.
- `make autoware-build`: passed on 2026-08-22. The only stderr was the existing
  setuptools deprecation warning.
- `ctest --test-dir /aichallenge/workspace/build/multi_purpose_mpc_ros
  --output-on-failure`: 33/33 passed.
- Focused execution-contract tests: 9/9 passed.
- Focused execution-orchestrator tests: 60/60 passed.
- Host `pre-commit` was unavailable (`command not found`). The package's CTest
  suite, including its configured lint tests, passed instead.

## Behavior-neutral review

- No parameter YAML, launch file, ROS topic, service or message contract was
  changed.
- No final-control source precedence or Ackermann command calculation was
  changed.
- The new contract is observation-only. Existing noncanonical paths are
  labelled `legacy-normal-bypass`; they are not rejected in Slice 1.
- A retained Dynamic Escape solution keeps the original problem fingerprint
  and solution ID instead of claiming to be a newly solved command.

## Runtime acceptance trial

Run a short `make dev2` trial for one to two laps. This is an identity and
joinability check, not a performance comparison.

Pass conditions:

1. Published commands have `contract_join=1`.
2. No published command reports `identity=incomplete`.
3. Emergency, Recovery and disabled outputs identify the explicit override
   authority without inventing a solver solution.
4. A retained solution, if exercised, reports `retained=1` and preserves its
   earlier problem fingerprint and solution ID.
5. Current legacy paths may report `canonical=violated`; this is expected
   migration evidence and must not itself alter driving behavior.

Suggested checks after the run:

```bash
rg -n "MPCC execution contract:" output/latest/d1/autoware.log
rg -n "contract_join=0|identity=incomplete" output/latest/d1/autoware.log
```

If the run output is stored in a timestamped directory rather than
`output/latest`, use that run's `d1/autoware.log` path.

## Runtime result: 20260822-120512

The short two-vehicle trial passed the Slice 1 identity gate.

| Domain | Contract traces | `contract_join=1` | `identity=complete` | Retained |
|---|---:|---:|---:|---:|
| d1 | 134 | 134 | 134 | 48 |
| d2 | 23 | 23 | 23 | 0 |

- There were no `contract_join=0` or `identity=incomplete` records.
- d1 classified 125 normal outputs as `legacy-normal-bypass`, three as
  Emergency overrides and six as Recovery overrides.
- d2 classified 22 normal outputs as `legacy-normal-bypass` and one as an
  Emergency override.
- d1 observed 83 five-state, 12 progress-contouring three-state, 35 legacy
  spatial-MPC and four unresolved-formulation trace transitions.
- Retained d1 executions preserved their earlier problem fingerprint and
  solution ID while reporting `retained=1`.
- No certified normal output was observed. This is expected Slice 1 migration
  evidence: current five-state output can be solved but is not yet the sole
  normal authority carrying one complete physical certificate.

The identity instrumentation showed no material callback regression. The d1
peak callback maximum was 46.035 ms versus 50.874 ms in the preceding
`20260822-105057` run; d2 was 20.360 ms versus 29.431 ms. d1 still has
pre-existing intermittent 25 ms overruns, which must be budgeted and measured
per stage in Slice 2, but this short comparison does not attribute an added
regression to the fingerprint contract.

The startup stale-odometry Emergency records were explicitly identified and
did not invent solver identity. Shutdown included RViz termination and stale
odometry after the run stopped; these are outside the Slice 1 identity gate.

### Exit decision

Slice 1 is accepted. Proceed to Slice 2 (`Track/Cruise` five-state MPCC shadow)
without changing production command authority or tuning parameters.
