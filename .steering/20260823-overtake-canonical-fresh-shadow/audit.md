# Root-cause audit

## Observed phenomenon

The stopped-front replay initially solves five-state Dynamic Escape, then OSQP reaches its iteration
limit near side-by-side geometry. The code immediately solves and publishes the three-state problem.
Circuit-open and requalification cycles continue publishing that other formulation.

## Earliest architectural violation

A successful five-state Overtake solution is not converted into the canonical execution artifact
used by Track/Cruise and Follow. Instead it is flattened into the old state/input layout before
prediction, post-processing and publication. Thus a physical solver result exists, but no canonical
plan or command identity exists at the final decision boundary.

## Propagation

```text
five-state solve succeeds
  -> lossy compatibility conversion
  -> final command has solution but no canonical command/plan identity
  -> final contract reports legacy-normal-bypass

five-state solve later fails
  -> circuit breaker/reentry gate
  -> three-state solve becomes normal authority
  -> authority/formulation oscillation
  -> wall handoff and output ownership churn
```

## Why tuning is not the first fix

Even a perfectly tuned five-state solver would still publish through the compatibility layout and
violate the canonical identity contract. Solver availability affects how often the second defect is
seen, but it is not the reason two normal authorities exist.

## Dynamic root-cause result

The bounded stopped-front replay evaluated 405 solved-cycle observations. Of those, 385 were live
Overtake intents with complete execution context, 353 satisfied the exact lateral row contract,
352 normalized and extracted exact actuation/trajectory, and all 352 passed the swept physical wall
certificate. The direct and canonical first actuation difference remained zero.

All 352 physically certified artifacts were rejected by the canonical plan contract before cursor,
candidate or command construction. This is not a solver or wall-certificate failure. The canonical
normal-intent contract currently permits only `Track`, `Cruise` and `Follow`; the shadow is limited
to `ShiftOut`, `Pass` and `Return`. Therefore every otherwise complete Overtake plan is rejected as
`UnsupportedIntent` by construction.

The earliest root cause for Gate A is consequently the incomplete canonical intent domain, not
`e_lag`, wall margin or OSQP tuning. Extending that domain must be a separate contract Slice with
tests for exact-intent matching. It must not promote production authority in the same change.
