# Results

## Frozen observation

The two independent ShiftOut wall-refinement failures were:

| run | interaction fingerprint | production result |
|---|---|---|
| `20260829-070314` | `a6f7c37f1de517c1` | OSQP maximum iterations |
| `20260829-073658` | `145d1159f38a6ea9` | OSQP maximum iterations |

The preceding Slice had already proved that removing either the artificial
lag bucket or the artificial heading bucket makes the affine set nonempty.
This Slice did not change any physical wall, opponent, terminal, tolerance,
iteration, clearance, Mission or production-authority contract.

## Independent backend result

With only the artificial lag bucket omitted, independent convex backends
reached the same racing optimum on both snapshots:

| backend | `145d...` | `a6f...` | physical affine residual |
|---|---:|---:|---:|
| OSQP, current explicit transform, internal scaling 0 | max iterations | max iterations | iterate remained within physical row tolerance |
| OSQP, same explicit transform, internal scaling 10 | solved / 925 iter | solved / 1200 iter | normalized `0.000151` / `0.00144` |
| qpOASES | solved | solved | approximately machine precision |
| ProxQP | solved | solved | normalized `< 0.005` |
| HiGHS | solved | solved | approximately machine precision |

The independent optimum was not directly publishable.  Nonlinear execution
replay rejected its upper lateral wall bound by only about `0.20 mm` on
`145d...` and `0.13 mm` on `a6f...`.  This is a model/certificate boundary,
not evidence of physical infeasibility.

When the same bucket-relaxed QP was run through audit-only OSQP internal
scaling and the complete existing SQP/proof chain, both snapshots became
certified ManeuverBundles.  `145d...` needed no post-refinement correction;
`a6f...` needed one.  End-to-end audit compute time was approximately
`76--100 ms` and `116--134 ms`, depending on which bucket was omitted.

## Corpus falsification

The result above does **not** justify promoting either bucket omission or
enabling OSQP internal scaling globally.

Across all 18 frozen ShiftOut failure snapshots from `20260829`, with audit-
only internal scaling and unchanged exact proofs:

| accepted arms per snapshot | count |
|---|---:|
| neither bucket omission accepted | 8 |
| only heading omission accepted | 3 |
| only lag omission accepted | 3 |
| both accepted | 4 |

Per arm, heading omission produced 7 accepted bundles, 3 exact dynamic-proof
rejections and 8 solver rejections.  Lag omission produced 7 accepted
bundles, 4 exact dynamic-proof rejections and 7 solver rejections.  Mean
offline evaluation time was about `101 ms` and `91 ms`, respectively.

Several exact dynamic-proof failures occurred although the affine disjunction
reserve was numerically nonnegative.  Representative cases had a full affine
disjunction reserve near `1e-7 m`, while the reconstructed physical footprint
already had negative clearance.  The certificate therefore disproved the
candidate model itself; a solver setting cannot repair it.

The earlier frozen Follow corpus also remains decisive: current explicit
scaling with OSQP internal scaling disabled solved Follow snapshot 5575,
whereas internal scaling 10 did not.  A deterministic explicit Ruiz-like
equilibration reproduced the same conflict: it solved both new ShiftOut
problems but regressed snapshot 5575.  A global scaling toggle is rejected.

## Root-cause classification

The current failure family has three coupled causes:

1. post-hoc wall refinement uses simultaneous hard lag and heading buckets;
   either bucket can empty the affine continuation depending on the current
   tangent;
2. racing objective conditioning is not uniformly compatible with one current
   OSQP scaling policy;
3. most importantly, the affine dynamic-obstacle disjunction is not equivalent
   to the exact swept physical-footprint certificate near its active boundary.

The third item explains why a numerically solved candidate can still direct
the vehicle into an opponent envelope.  It is a model/certificate mismatch,
not a Mission resume, lease, timeout, clearance or wall-margin problem.

## Architecture decision

Do not promote the audit solver policy, Phase-I objective, lag omission or
heading omission.  No production path changed.

The next Slice must make candidate constraints and the exact physical proof
describe the same occupied set.  It should first replay the frozen dynamic-
proof failures and compare:

1. the current affine disjunction reserve;
2. physical footprint clearance at the same affine nodes;
3. swept clearance between nodes;
4. a candidate generated from the physical separation geometry itself.

Only after the dynamic candidate is proof-consistent should wall refinement
be converted from hard pose buckets to one canonical sequential-convexification
trust-region design.  This ordering avoids tuning the solver around a candidate
that the physical certificate correctly rejects.

## Audit cleanup and verification

- all C++ external-primal bucket and internal-scaling hooks were deleted;
- production authority and solver settings are unchanged;
- offline scripts retain the immutable numerical evidence;
- `make autoware-build` passed while the audit hooks were present;
- the final source diff contains steering evidence only.

Primary numerical references consulted during the bounded audit:

- OSQP settings: <https://osqp.org/docs/interfaces/solver_settings.html>
- OSQP algorithm: <https://osqp.org/docs/solver/>
- OSQP scaling source: <https://github.com/osqp/osqp/blob/master/src/scaling.c>
- ETH MPCC reference implementation: <https://github.com/alexliniger/MPCC>
- TUD-AMR topology-parallel MPC planner: <https://github.com/tud-amr/mpc_planner>
