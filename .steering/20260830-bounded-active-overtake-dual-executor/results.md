# Results

## Static verification

- `make autoware-build`: passed, 25 packages.
- package CTest: passed, 57/57 tests.
- bounded executor tests cover persistent-thread reuse, no-queue Busy
  rejection, job exceptions, invalid tickets, and stopped submission.
- source contract rejects `std::async` in the active Overtake dual evaluator.

## Dynamic verification

Run: `output/20260830-122030`

- active Overtake entered `ShiftOut`;
- four active dual telemetry windows were observed;
- a same-epoch state with both sides certified was observed;
- `bounded negative branch unavailable`: 0 occurrences;
- executor Busy/job-failed evidence: 0 occurrences;
- mean of active window-average background compute: 122.944 ms;
- maximum active background compute: 289.759 ms.

The preceding per-evaluation-thread run had a 471.697 ms maximum. The runs are
not identical, so the difference is not a performance benchmark, but the
bounded executor removed thread creation without introducing a larger observed
tail or losing branch evidence.

The encounter later entered Recovery for `actual footprint wall margin
violated`. That is an independent physical execution failure and confirms this
Slice did not hide authority outcomes.

## Conclusion

The active sibling producer now has bounded persistent execution suitable for
the next authority Slice. It still has observation-only authority. Exact
sibling adoption and Return authority continuity remain separate work.
