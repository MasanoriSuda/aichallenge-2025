# Results

## Static verification

- `make autoware-build`: passed, 25 packages built.
- package CTest: passed, 56/56 tests.
- `git diff --check`: passed.

The tests cover exact source identity, intent, side, epoch, stale replacement,
and atomic invalidation of both stored sides. The source contract verifies that
the opposite branch cannot select homotopy ownership or reach the publisher.

## Dynamic verification

Run: `output/20260830-120323`

Episode 1:

```text
Idle -> ShiftOut (side +1)
ShiftOut -> Pass
Pass -> Return
Return -> Idle
```

At Pass sequence 1601, both `-1` and `+1` branches were exact-certified. A
separate Return authority loss occurred after the Pass and remains outside the
ShiftOut/Pass evidence-bank scope.

Episode 2:

```text
seq=2100
intent=ShiftOut
selected=-1
selected_certified=0
sibling_certified=1
selected rejection=exact physical proof / invalid lateral bounds / stage 135

later:
ShiftOut -> FollowPrepare -> Idle
```

This is the required counterexample: the same current world admitted an exact
certified opposite homotopy before production abandoned the encounter. It
classifies the failure as production lifecycle/authority adoption, not physical
infeasibility.

Episode 3 later reached a selected-side authority loss and wall Recovery. Its
preceding bank states alternated between one and two certified sides, so it
must be analyzed separately; stale historical sibling evidence must not be
used to rescue it.

## Timing

- active telemetry windows: 11;
- mean of reported background-compute window averages: 95.613 ms;
- maximum reported background compute: 471.697 ms;
- current 25 ms callback remained separate, but overrun events still occurred.

The dynamic comparison validates the algorithmic evidence but rejects
per-evaluation `std::async` as the final production scheduler.

## Classification

- Frozen replay: selected A failed while opposite stateless/current-world
  evaluation succeeded.
- Live evidence: the same selected-vs-sibling split occurred before tactical
  abandonment.
- Classification: persistent Mission homotopy/authority adoption defect.
- Not established: all failures are sibling-rescuable.
- Separate open issue: Return normal-authority continuity.
