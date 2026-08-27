# Requirements

## Objective

Classify and remove the two structural failure modes captured by dynamic Gate
`output/20260828-032723` without tuning Mission lifetime, solver tolerance, or
physical clearance:

1. a solved-inaccurate QP passes the generic row certificate but its execution
   artifact violates the exact physical acceleration envelope;
2. wall-refined ShiftOut/Return QPs reach maximum iterations and must be
   distinguished from mathematical infeasibility.

## Constraints

- Keep production authority unchanged.
- Do not add a fallback, lease, grace period, or timeout.
- Do not change OSQP tolerance/iteration settings.
- Do not change wall, opponent, or acceleration physical limits.
- Every correction must remove a producer/certificate inconsistency and have a
  frozen-snapshot regression test.

## Definition of Done

- Frozen QPs have an independently reproducible linear-feasibility
  classification.
- Exact actuator insets cover every solver status accepted by the generic
  physical-row certificate.
- Package build and focused/unit contract tests pass.
- A new dynamic Gate confirms whether the classified failures recur.
