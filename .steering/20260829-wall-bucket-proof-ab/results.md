# Results

## Observed failure

Two independent live ShiftOut snapshots reached the same boundary:

| run | interaction fingerprint | production failure |
|---|---|---|
| `20260829-070314` | `a6f7c37f1de517c1` | wall-refined QP maximum iterations |
| `20260829-073658` | `145d1159f38a6ea9` | wall-refined QP maximum iterations |

The second run used unchanged production authority.  It repeatedly generated
ShiftOut candidates but could not publish a canonical Overtake artifact before
the ordinary downstream Recovery path.

## Bucket A/B

J omits only the artificial physical-refinement heading box.  K omits only the
artificial lag box.  Lateral/progress wall rows, swept footprint rows,
dynamics, actuator bounds, opponent constraints and exact proof gates are
unchanged.

For both ShiftOut fingerprints, both J and K solved the identity-projection
Phase-I problem.  Therefore the original simultaneous lag/heading bucket set,
not the physical wall rows, was making the recorded affine problem empty.

After restoring the exact racing objective, however, all four ShiftOut solves
reached 4,000 OSQP iterations:

| fingerprint | arm | Phase-I | racing solve | representative residual |
|---|---|---|---|---|
| `a6f7c37f1de517c1` | J omit heading | solved | maximum iterations | dual `0.210504` |
| `a6f7c37f1de517c1` | K omit lag | solved | maximum iterations | dual `0.292712` |
| `145d1159f38a6ea9` | J omit heading | solved | maximum iterations | dual `0.0200192` |
| `145d1159f38a6ea9` | K omit lag | solved | maximum iterations | dual `6.17987` |

Before the racing objective was deliberately restored, the K Phase-I
trajectory on each ShiftOut snapshot passed the unchanged exact wall, dynamic
obstacle and terminal-successor proofs.  That is a physical-feasibility
witness, not a production candidate.

The audit is not universally unable to optimize the racing cost.  On frozen
Follow wall fingerprint `100d4259e9970705`, J solved the original racing
objective and passed all exact proofs with terminal velocity `3.16239 m/s`.
Conversely, frozen Follow dynamic-obstacle fingerprint `52ddd305be5f3b6d`
still failed its dynamic-obstacle row under both J and K; wall buckets are not
causal for that separate failure.

## Numerical boundary

The new ShiftOut racing QP has 207 variables.  Under the current explicit box
coordinate transform its positive Hessian spectrum is approximately
`0.0014231` to `8771.37`, a condition ratio of about `6.16e6`.  The weakest
diagonal is the steering-rate input while lateral/heading state coordinates
carry values up to roughly `8771` and `5000`.  A physically feasible warm
start therefore removes primal infeasibility but does not remove the observed
dual-convergence boundary.

This does not justify reversing the earlier internal-scaling correction.  The
existing `20260828-feasible-qp-conditioning-audit` proved on a different six-
snapshot corpus that OSQP's internal scaling regressed some already explicitly
conditioned QPs and helped none exclusively.  The present evidence is a new
formulation/backend boundary after that correction.

OSQP's own implementation normally performs iterative Ruiz equilibration over
the KKT columns and rows and separately normalizes the objective.  Its default
setting is ten scaling iterations.  Those primary sources explain why the
current Hessian/constraint imbalance is a legitimate audit dimension, but do
not establish a safe production setting for this mixed explicitly scaled
problem:

- https://osqp.org/docs/interfaces/solver_settings.html
- https://github.com/osqp/osqp/blob/master/src/scaling.c
- https://osqp.org/docs/solver/

The ETH reference MPCC instead uses HPIPM for the time-varying structured QP,
keeps track boundaries as half spaces in the optimization, and uses a coarse
dynamic-programming obstacle corridor before refinement:

- https://github.com/alexliniger/MPCC

The local upper-rank trace in `.steering/ano` is consistent with a continuously
solved main GMPCC (`N=20`, `dt=0.12`, `nvar=349`, `ncon=818`) and asynchronous
tactical work.  It normally reports solved main iterations in roughly
25--55 ms and exposes nonzero `sb`/`sc` fields.  The source is unavailable, so
those fields are treated only as evidence of a different numerical/elastic
architecture, not as a decoded implementation contract.

## Root cause classification

The frozen ShiftOut failure is not a Mission-lifecycle defect and is not
physical wall infeasibility.  It is a coupled two-part formulation defect:

1. post-hoc wall refinement imposes simultaneous hard lag and heading buckets
   which can exclude every affine continuation even though a physically
   certified trajectory exists;
2. after removing one artificial bucket, the current racing QP and explicit
   coordinate scaling remain poorly matched to the OSQP ADMM backend, so a
   feasible primal still does not converge to the racing optimum in the fixed
   runtime budget.

## Decision

Do not promote J, K or their Phase-I objective.  Do not change clearance,
tolerance, iteration count, Mission lifecycle, fallback, timeout or authority.

The next bounded Slice must replay these exact ShiftOut worlds and compare the
same hard physical problem under numerically traceable formulations/backends:

1. current explicit box scaling and OSQP;
2. objective/KKT-aware explicit coordinate scaling with exact physical-row
   certificates unchanged;
3. an independent active-set or structured OCP-QP solver offline.

If an alternate numerical formulation solves the racing objective and passes
the existing exact proofs on both ShiftOut fingerprints, production promotion
must atomically remove the simultaneous hard-bucket path rather than install a
second normal fallback.  Otherwise the audit-only bucket entry points are
deleted.

## Verification

- package build: succeeded after removing one stale generated symlink-install
  directory; no source workaround was added;
- focused GTest/source-contract run: `1960 tests`, zero failures;
- frozen replays: both ShiftOut fingerprints, one Follow wall fingerprint and
  one Follow dynamic-obstacle fingerprint completed;
- production command path: unchanged and protected by a source-contract test.
