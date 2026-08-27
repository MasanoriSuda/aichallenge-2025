# Validation

## Static Gate

- `make autoware-build`: passed, 25 packages.
- `test_single_authority_source_contract`: passed.

## Dynamic A/B

- A: `N=20`, `output/20260828-044759`.
- B: `N=16`, `output/20260828-045652`.

The B run reduced the observed computation tail: maximum MPCC cycle time was
47.360 ms instead of 56.310 ms, and total overruns in the bounded run were 46
instead of 102 in the longer baseline run.  Mean one-second-window MPCC time
was essentially unchanged (3.676 versus 3.810 ms), so the benefit was limited
to the tail.

The dynamic execution regression is decisive.  B produced only one
`ShiftOut -> Pass`, no `Pass -> Return -> Idle`, and introduced:

```text
ShiftOut -> FollowPrepare
  reason=dynamic Mission wait: Pass entry physical wall gate unresolved
FollowPrepare -> Recovery
  reason=dynamic Mission wait has no wall-feasible lateral authority
```

The baseline produced `ShiftOut -> Pass -> Return -> Idle` and no
ShiftOut/Pass Recovery.  The 16-stage horizon can no longer prove a viable
wall-feasible successor in the same encounter class.  This violates the
experiment acceptance criteria.

## Decision

Rejected.  Both local and cloud configurations are restored to `N=20`.
Weights, clearances, solver tolerances, and authority were not changed.  A
separate conservative `N=18` experiment may test whether a smaller compute
tail reduction is available without losing terminal successor viability.
