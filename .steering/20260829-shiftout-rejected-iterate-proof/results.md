# Results: ShiftOut rejected-iterate proof

## Frozen observation

The immutable source is interaction fingerprint `9845010060330222052` from
run `20260829-133704`.  Production reached OSQP's 4000-iteration boundary on
the ShiftOut wall-refinement QP.  The reported primal residual was
`0.000485735`, the dual residual was `0.0790199`, and the worst physical row
had normalized violation `0.43103`.

This Slice changed no production authority, QP, solver setting, clearance,
timeout, lease, fallback or command path.

## Rejected iterate classification

`PersistentOsqpSolver` already reconstructed the finite physical-coordinate
iterate for diagnostics but discarded it.  The new diagnostic-only field
preserves that vector while `SolveOutcome::result` remains empty.

Replaying the frozen problem and passing the rejected iterate through the
unchanged exact proof chain produced a certified observation-only bundle:

- terminal progress: `15.8743 m`;
- terminal velocity: `7.15275 m/s`;
- minimum lateral reserve: `0.0090652 m`;
- physical affine normalized violation: `0.43103`;
- exact nonlinear trajectory, swept wall, timed obstacle and terminal
  successor proofs: accepted.

Therefore the failure is not physical infeasibility, Mission lifecycle loss
or candidate absence.  It is a numerical/KKT termination mismatch: a finite
physically certifiable iterate is discarded because the ADMM dual residual
has not reached the solver's convergence status.

## Independent numerical comparison

The identical recorded QP was solved offline without changing any physical
row or objective.  Only the explicit coordinate equilibration differed:

| formulation | status | iterations | primal residual | dual residual |
|---|---:|---:|---:|---:|
| current box/row scaling | maximum iterations | 4000 | `2.27925e-4` | `0.609411` |
| one Ruiz pass | maximum iterations | 4000 | `2.41607e-4` | `7.27152e-4` |
| three Ruiz passes | solved | 1150 | `6.16268e-6` | `1.23251e-4` |
| ten Ruiz passes | solved | 1125 | `4.63858e-6` | `1.33933e-5` |

Both converged Ruiz primals passed the same exact C++ proof chain.  The
three-pass result reached `15.875 m`, `7.16672 m/s` and `0.00946819 m`
lateral reserve.  The ten-pass result reached the same progress and velocity
with `0.00946627 m` reserve.

This independently confirms a QP coordinate-scaling/backend mismatch.  It
does not authorize accepting maximum-iteration results or enabling a second
production fallback.

## Evidence-platform change

- maximum-iteration finite primals can be serialized in architecture
  snapshots as rejected evidence;
- old snapshots remain loadable and can reconstruct the evidence by exact-QP
  replay;
- `mpcc_architecture_compare --rejected-primal-only` checks the exact recorded
  affine rows and every existing physical proof;
- the historical equilibration audit can optionally export physical-coordinate
  primals for the same proof chain.

## Verification

- `make autoware-build`: 25 packages completed successfully;
- focused CTest: four suites, zero failures;
- rejected production iterate: exact proofs accepted;
- independently converged three- and ten-pass Ruiz primals: exact proofs
  accepted.

## Decision

Close this observation Slice as `accepted`.  The next bounded Slice may
evaluate a canonical KKT-aware coordinate formulation in shadow/offline mode.
Promotion is permitted only if it replaces the current normal formulation in
one authority path, preserves every exact proof, and is falsified across the
frozen corpus plus dynamic ShiftOut/Pass runs.  It must not be installed as a
retry, fallback or relaxed solver-status rule.
