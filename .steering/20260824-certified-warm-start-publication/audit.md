# Audit

## Observed chain

```text
OSQP solved
  -> raw primal/dual stored immediately
  -> execution-primal semantic check rejects bound
  -> emergency authority
  -> next solve shifts the rejected raw artifact
  -> warm-only reject chain
```

## Evidence

- Baseline `output/20260824-005436`: D2 had 153 execution-primal rejects.
- Dual-boundary experiment `output/20260824-013035`: D2 still had 8 rejects
  in the bounded sample; every reject had `warm=1`.
- Rejected fields were acceleration (1) and predicted velocity (7), so the
  failed dual-only hypothesis does not explain the chain.
- Source inspection shows the unconditional store at the end of
  `solve_extended_progress_problem()`, before all three canonical fresh paths
  call `normalize_extended_execution_primal()` and physical certification.

## Invariants

- Solver success is not certification.
- Warm history contains only a normalized, downstream-accepted artifact.
- Time, progress origin, primal and dual share one lifetime.
- Consuming a warm artifact is single-use; rejection cannot reuse it.

## Verification

- Build: 25 packages succeeded.
- Tests: 40/40 programs, 1702 tests, 0 failures.
- Bounded runtime: `output/20260824-014849`.
- D2 produced 165 certified and 168 execution-primal-rejected outcomes.
- The former persistent reject chain became an exact alternating sequence:
  every rejected cycle had `warm=1`, and every immediately following cycle
  was a certified `warm=0` solve. No publication rejection occurred.

This proves that rejected artifacts no longer persist in warm history. It also
exposes the next independent defect: the current one-stage warm transform is
itself unusable at 40 Hz. Of the 168 rejected warm results, 151 violated stage
zero acceleration, 16 predicted velocity and one virtual progress speed. The
next Slice must make warm transport elapsed-time/stage-time aware; it must not
undo this accepted-only publication boundary or hide the failures with a
cold-retry policy.
