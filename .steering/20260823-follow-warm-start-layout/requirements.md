# Follow warm-start constraint-layout repair

## Purpose

Restore receding-horizon warm starts for the Follow 5-state MPCC QP without weakening the physical
`progress + lag` hard-gap rows added in the previous slice.

## Root cause

The generic warm-start shifter accepts only the original dual layout. Follow appends one typed
`N + 1` state-stage constraint block, so the dual-size check rejects every prior solution and every
numeric update is cold-started.

## Constraints

- Do not change OSQP settings, wall/gap margins, driving parameters, or production authority.
- Do not infer unknown trailing rows from vector length.
- The QP builder must declare every additional dual block explicitly.
- Legacy problems without extra rows must retain their exact behavior.

## Acceptance

- Base warm-start layout still shifts identically.
- A declared `N + 1` Follow gap block shifts stage-wise.
- Extra undeclared or malformed rows are rejected.
- Package tests and build pass.
- A later `make dev2` run shows nonzero Follow shadow warm starts while remaining
  `authority=shadow, selected=0`.
