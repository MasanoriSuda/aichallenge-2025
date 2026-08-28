# Results: Native A/B failure classification

## Native evidence

- Run: `output/20260828-094214`
- Domain: 1
- Decision/sequence: 2172 / 1566
- Intent: Pass, positive homotopy
- Immutable interaction fingerprint: `7246006054995400977`
- Failure: wall-refined seven-state SQP reached OSQP maximum iterations.

The first observation exposed two comparison-harness defects before the
solver: world generation was compared with ego decision generation, and valid
unbounded semantic rows were rejected as non-finite.  Those are snapshot
contract defects, not control-policy findings, and were repaired before the
architecture comparison.

The next comparison exposed two more architecture-boundary defects: B still
read Mission-derived target stages, and all arms required a short horizon to
finish Return or reach a zero-velocity terminal interval.  The sibling
`20260828-current-world-ab-contract` Slice repairs those non-production
comparison contracts.

## Frozen comparison

| Arm | Numerical result | Exact proof | Detail |
|---|---|---|---|
| A persistent | failed | not run | maximum iterations; original failed row and normalized violation reproduced |
| B positive side | solved in 35.27 ms | failed | target `d2`, minimum swept clearance `-0.007975 m` |
| B negative side | failed | not run | maximum iterations; lateral state-box violation `0.007659 m` |

The registered classification is `model_certificate_mismatch`: the positive
stateless current-world candidate is numerically feasible to the convex QP but
fails the common dense physical opponent proof by about 8 mm.  This does not
open production authority.  It also does not support a Mission-lifecycle-only
root cause, because B did not produce a certified ManeuverBundle.

## Upper/reference evidence

`.steering/ano/autoware - 2026-08-21T211659.829.log` reports a GMPCC with
`N=20`, `dt=0.12`, a 2.4-second horizon and asynchronous tactical evaluation.
The main GMPCC log continues to carry opponent relation every solve.  It does
not show a requirement that Return or complete Stop be present inside each
short horizon.

The external architecture check is consistent with:

- `alexliniger/MPCC`: obstacle-aware DP/corridor generation above MPCC;
- `tud-amr/mpc_planner`: parallel homotopy/topology candidates with dynamic
  obstacle constraints;
- game-theoretic racing MPC: opponent prediction and high-level strategy are
  separate from low-level MPC.

References:

- https://github.com/alexliniger/MPCC
- https://github.com/alexliniger/MPCC/issues/3
- https://github.com/tud-amr/mpc_planner
- https://arxiv.org/abs/2106.04094

## Decision

Do not tune clearance, solver tolerance or iterations.  The next bounded audit
must localize where the sparse convex opponent model permits the 8 mm dense
physical overlap.  Candidate C/D is necessary only if that mismatch cannot be
explained or removed at the shared model/certificate boundary.

