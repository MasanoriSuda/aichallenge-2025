# Audit

## Hypotheses

| Hypothesis | Supporting evidence | Refutation | Confidence |
|---|---|---|---:|
| Shifted dual crosses constraint semantics at stage zero | Generic shift maps transition to measured equality and delta rate to absolute rate | Zeroing both category-changing boundaries did not prevent warm-start execution-primal rejects | rejected |
| A post-certification reject remains saved as the next warm start | Every observed reject was warm-started; `solve_extended_progress_problem()` stores the OSQP result before the caller's semantic execution-primal normalization | Move/withhold publication of warm state until semantic acceptance and observe no reject chain | high |
| Warm primal rebase is the primary defect | All observed strict and production-policy failures are warm-started | Accepted-only warm-start publication removes reject chains without changing primal rebasing | medium |
| Row-normalized policy alone fixes Track/Cruise | It catches local residuals | 32/65 and 9/19 outcomes failed | rejected |
| Retained expansion should be first | It could bridge fresh misses | Early retained windows are themselves invalid and would mask the numerical producer | rejected |

## Invariants

- Warm start is only an initial guess; it never becomes an executable plan.
- Solver and post-solve physical certification remain authoritative.
- A row without a same-semantic predecessor receives a neutral dual, not an
  invented multiplier.

## Dynamic refutation

- Build: 25 packages succeeded.
- Tests: 1700 tests, 0 errors, 0 failures.
- Bounded run: `output/20260824-013035`.
- D2 outcomes: 2 certified, 8 execution-primal rejects.
- All 8 rejects used `warm=1`; fields were acceleration (1) and
  predicted-velocity (7).
- Therefore the proposed dual-boundary clearing did not resolve the observed
  reject chain and is not retained in production code.
