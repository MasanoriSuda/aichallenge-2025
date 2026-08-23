# Requirements

## Objective

Repeat the live Overtake fresh/retained authority Gate after the five-state
numerical-contract repair in `8ff1a9e`. Identify the earliest uncovered
canonical Overtake cycle before any further source or parameter change.

## Scope

- Freeze source/config at `8ff1a9e`.
- Run `make dev2` and require a real Overtake interval.
- Aggregate fresh/retained selection, rejection reasons, solver/physical
  failures, final authority and callback timing.
- Select the next root-cause Slice from the earliest violated invariant.

## Non-scope

- No authority promotion in this observation Slice.
- No timeout, lease, fallback, retry, flag or parameter tuning.
- No Overtake behavior patch before a live causal interval exists.

## Acceptance

The run must contain a Domain 1 Overtake interval with typed canonical outcome
and final-authority evidence. If none occurs, the Gate remains incomplete.
