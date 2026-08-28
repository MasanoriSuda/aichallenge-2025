# Results

## Static verification

- `make autoware-build`: 25 packages passed.
- package tests: 54/54 targets, 2089 assertions, 0 failures.
- single-authority source contract: 70 passed.
- focused retained test proves both sides of the boundary:
  - production still returns `SteeringUnreachable`;
  - the observation-only projected command builds an exact nonlinear
    continuation.

The feedback module cannot create a production candidate, publish a command or
mark an artifact executed.

## Dynamic run

- Run: `output/20260829-031339`
- Mode: bounded `make dev2`
- Production authority, solver policy, wall clearance and behavior parameters
  were unchanged.

The aggregate counters count current-world candidate evaluations, not unique
solver artifacts.

| Domain | feedback attempted | projected | nonlinear continuation | rate |
|---|---:|---:|---:|---:|
| d1 | 2933 | 2933 | 2863 | 97.6% |
| d2 | 215 | 215 | 213 | 99.1% |
| total | 3148 | 3148 | 3076 | 97.7% |

One visible failed feedback continuation was classified as
`actuator-envelope-rejected`; no tolerance was changed to hide it.

## Causal failure trace

Domain 1 continued to generate new certified candidates, but many candidate
evaluations were rejected as `steering-unreachable`.  The last actually
executed plan remained sequence 626 until its cursor was exhausted.  The
measured progress then remained at 28.773461 m and production repeatedly chose
`canonical-normal-emergency-stop` because rate-resolved authority was
unavailable.

In the same windows, the rejected candidate steering was projectable into the
unchanged publication slew envelope and the existing nonlinear model accepted
the corrected continuation.  For example, the final d1 window classified all
81 feedback attempts as projected and all 81 as nonlinear-continuation
available while the old executed cursor remained exhausted.

## Root-cause classification

This is a **latest-state feedback/certification connector defect** in the live
asynchronous scheduling path.  It is not evidence for changing wall clearance,
solver tolerance, Mission lifetime, lease, grace period or fallback behavior.

The previous exact parent/candidate connector was too strict for feedback
control, while elapsed suffix selection alone was non-causal.  An analytical
bounded feedback command reconnects almost all observed candidates to the
current actuator state, but it is not yet a complete authority proof because
the corrected trajectory has not passed the current wall, timed dynamic
obstacle and terminal-successor certificates.

## Decision

Proceed with one more observation-only certification Slice:

1. form a corrected current-world trajectory from the feedback command;
2. run the unchanged wall and timed dynamic-obstacle proofs;
3. validate terminal successor viability;
4. preserve the original candidate and world fingerprint;
5. measure runtime and acceptance without publishing it.

If that complete proof succeeds dynamically, production promotion must create
one new feedback-certified artifact and atomically delete elapsed-suffix-only
candidate adoption.  If it fails, classify the first exact proof boundary
instead of adding a clamp or timing exception.
