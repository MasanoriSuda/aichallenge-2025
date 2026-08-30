# Results: current-world proof runtime audit

## Verification

- `make autoware-build`: passed.
- `multi_purpose_mpc_ros`: 2,247 tests, 0 failures.
- Dynamic runs:
  - `output/20260831-004453`
  - `output/20260831-005930`
  - `output/20260831-010730`
- The first run completed one full
  `ShiftOut -> Pass -> Return -> Idle` episode.

## Measured proof cost

Across the first bounded run, the selected retained proof regions were:

| Domain | Samples | Continuation proof avg/max | Terminal wall avg/max |
|---|---:|---:|---:|
| d1 | 47 | 1.971 / 13.652 ms | 0.945 / 5.143 ms |
| d2 | 50 | 2.571 / 14.246 ms | 0.615 / 3.034 ms |

Ordinary accepted evaluations usually consumed one plan and approximately
2--6 ms.  An occasional single full-suffix proof reached approximately
20 ms, but it did not explain the 33--79 ms production-join tail.

## Root cause

The dominant tail is the observation-only terminal-contingency failure
snapshot recorder executing synchronously in the 25 ms control callback.

Two frozen examples from `output/20260831-010730/d1/autoware.log`:

1. Decision 1380, ShiftOut:
   - production total: 29.696 ms
   - primary retained proof: 1.227 ms
   - Stop-lattice join: 2.433 ms
   - failure snapshot: **25.583 ms**
2. Decision 1894, Cruise:
   - production total: 79.454 ms
   - primary retained proof: 0.308 ms
   - Stop successor evaluation/join: 1.430 / 2.304 ms
   - failure snapshot: **68.999 ms**

The recorder reconstructs a solver/physical/replay snapshot and writes a
large YAML plus a roughly 570 kB wall-grid file before the normal command can
return.  It is diagnostic only, but its disk and serialization work therefore
delays publication beyond the certificate's own interval.  The late next
cycle can then invalidate a previously viable Stop suffix and propagate to
external Stop.  This is a scheduling/observability ownership defect, not a
clock, velocity tolerance, wall-margin or SQP-parameter defect.

## Rejected fixes

- Changing candidate/published execution clocks.
- Relaxing velocity, wall or steering reachability.
- Extending the certificate lease or adding a grace period.
- Tuning OSQP/SQP parameters.

None address the measured synchronous diagnostic I/O.

## Next Slice

Move architecture failure-snapshot serialization and filesystem I/O to one
bounded latest-only observation worker.  Production may capture immutable
evidence, but it must not wait for YAML/wall-grid persistence.  The worker
must not own authority, alter command selection or create a fallback.

Acceptance requires:

- snapshot files still appear for terminal-contingency failures;
- production `failure_snapshot` time falls below the publisher budget with
  no 25--69 ms disk-I/O tail;
- 2,247 static tests remain green;
- a bounded dev2 run retains a complete Overtake chain and reduces callback
  overruns attributable to snapshot recording.
