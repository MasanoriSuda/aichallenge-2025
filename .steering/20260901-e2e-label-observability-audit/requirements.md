# Requirements

## Objective

Determine whether material teacher corrections and production-normal zero
corrections are distinguishable under the exact v11 model input contract.

## Constraints

- production model/runtime remains frozen;
- compare immutable teacher and admitted production-normal sequences;
- preserve run identity and exclude same-run neighbours when deriving the
  natural-distance baseline;
- use the candidate's normalized spatial features, synchronized wheel speed and
  embedded base steering exactly as the adapter sees them;
- report evidence without changing labels, model weights or admission Gates.

## Definition of Done

- a reusable diagnostic computes cross-corpus nearest-neighbour conflict;
- tests cover deterministic sampling, same-run exclusion and conflict metrics;
- the current corpus is audited and classified;
- the next data/model action is derived from the result.
