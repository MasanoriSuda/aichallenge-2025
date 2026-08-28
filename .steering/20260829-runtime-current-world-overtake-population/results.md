# Results: runtime current-world Overtake population

## Static result

- The canonical normal worker now has one dispatcher.
- Follow continues to use the bounded Follow escape population.
- ShiftOut, Pass and Return use the selected-side current-world Overtake
  population and cannot fall through to the persistent direct solver.
- Track, Cruise and Rejoin keep their existing canonical direct pipeline.
- Missing Overtake physical evidence is a typed build rejection.

The complete `multi_purpose_mpc_ros` package test suite passed: 54 CTest
targets, 2,100 tests, zero failures, errors or skips.

## Dynamic gate

Run: `output/20260829-061205` (`make dev2`).

The run did not reach an Overtake intent. Both vehicles remained in Cruise
because canonical Cruise never obtained its first production artifact:

```text
fresh worker result: solved + exact physical wall accepted
candidate store: advances
retained join: steering-unreachable
production: retained-proof-unavailable
publisher: canonical-normal-emergency-stop
vehicle: remains at 0 m/s
```

The retained diagnostics simultaneously showed that latest-state steering
projection and its unchanged exact continuation proof were often accepted.
This excludes wall infeasibility and failure to solve the fresh QP for the
cold-start observation.

## Classification

The Overtake change is not the cause: Cruise follows the unchanged direct
pipeline in the new dispatcher. The blocker is an earlier lifecycle regression
in `UnpublishedCandidate` execution-clock semantics. A candidate which has
never crossed the publisher is currently sampled at the elapsed suffix from
its prediction origin, even when no executed predecessor exists. The consumer
therefore requests steering from an unexecuted prefix and rejects it against
the actual zero command.

Dynamic Overtake acceptance remains pending until the independent Cruise
bootstrap contract is repaired. No Overtake fallback, threshold, lease,
timeout or clearance change was added in response.
