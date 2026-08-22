# Validation status

## Static status at `5bae30b`

- Build: 25 packages passed.
- Complete package CTest: 35/35 passed.
- Test result: 1,571 tests, 0 errors, 0 failures, 0 skipped.
- Current-intent failure-first tests demonstrated and then rejected Track/Cruise cross-intent
  retained adoption and non-Track/Cruise current requests.
- Canonical plan actuation is read only by Track/Cruise shadow evaluation.
- The final command publisher remains unchanged.
- User-owned `aichallenge/result-summary.json` remains uncommitted and untouched.

## Dynamic Gate A status

- `output/20260822-232351`: one-car `make dev`, five observed waypoint wraps.
- 9,795 eligible/solved shadow cycles.
- 9,678 physically certified cycles; all 9,678 reached canonical extraction, storage, cursor,
  candidate, fresh selection and exact actuation extraction.
- `actuation_diff=0`, `authority=shadow`, `selected=0` in every reported window.
- Three non-executable virtual-progress results were rejected before certification.
- Two callback overruns in 256 one-second windows; maximum 28.138 ms. Shadow maximum was
  13.541 ms, so no consecutive/stale-command failure was attributed to the shadow solve.

Gate A is accepted for beginning retained **shadow** implementation. It does not approve final
publisher promotion.

## Rejected shared-solver experiment

`output/20260822-234326` tested strict per-row admission in the shared OSQP wrapper. It rejected
legacy MPC curvature-rate row 166 at the first start and cascaded into repeated maximum-iteration
failures. The source/test experiment was removed; see
`.steering/20260822-osqp-rowwise-residual-admission`.

## Required next artifact

Implement the pure retained revalidation contract, connect it to shadow selection only, then run a
fresh-miss scenario that proves current pose, wall geometry and obstacle generation are revalidated
before a retained stage is accepted.
