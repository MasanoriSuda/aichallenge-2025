# Track/Cruise canonical fresh admission

## Baseline

- Branch: `develop_july`
- Baseline commit: `c60f9ce`
- Preserve `aichallenge/result-summary.json`.

## Missing proof

The complete certified shadow plan is retained, but no runtime path proves that the plan, exact
cursor, current physical certificate and canonical selector agree. Solver/store coverage alone
cannot justify authority promotion.

## Required correction

- Resolve an exact cursor from the just-stored immutable plan.
- Bind the current decision's physical proof to the exact plan ID and remaining window.
- Build a fresh canonical candidate through the production contract.
- Evaluate the canonical authority selector in shadow mode.
- Treat any cursor, candidate or selector disagreement as an explicit shadow reject.
- Preserve `selected=0` and all existing final command ownership.

## Exit gate

- A certified shadow outcome implies `FreshCertified` shadow admission.
- Telemetry distinguishes solve, store and admission coverage.
- No read of canonical controls by the publisher.
