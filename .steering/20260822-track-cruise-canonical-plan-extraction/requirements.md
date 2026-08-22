# Track/Cruise canonical plan extraction

## Baseline

- Branch: `develop_july`
- Baseline commit: `56d8a39`
- Preserve `aichallenge/result-summary.json`.

## Root cause addressed

The complete-plan store exists, but the Track/Cruise shadow solver currently exposes only a
first-stage actuation proposal and a legacy-converted 3-state/2-input vector. Neither is a valid
source for the canonical store:

- first-stage extraction discards the remaining executable sequence;
- legacy conversion discards lag, acceleration and virtual progress input semantics;
- reconstructing a canonical plan from either would invent data and break problem/solution
  provenance.

## Required correction

- Extract all `N + 1` `[e_y, e_lag, e_psi, v, theta]` states directly from the certified primal.
- Restore absolute progress exactly once from the local progress origin.
- Extract all `N` `[a, kappa, v_theta]` inputs without legacy conversion.
- Bind exact stage durations, plan ID, problem fingerprint, solution ID and solve time.
- Reject malformed size, non-finite values, missing stage timing and incomplete metadata.
- Validate the resulting complete plan before returning it.

## Non-scope

- No mutation of the runtime plan store.
- No controller include/reference to the adapter.
- No command or parameter change.
- No authority promotion.

## Exit gate

- Known primal values round-trip into the canonical plan without semantic flattening.
- Malformed inputs fail with explicit reasons.
- Build and complete tests pass.
