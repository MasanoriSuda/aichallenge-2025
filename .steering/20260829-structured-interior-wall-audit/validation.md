# Validation: structured interior-wall audit

## Frozen replay

All arms retain the same snapshots, 25 ms latest-state probe, costs, bounds,
solver limit and exact physical proof. The structured arm replaces all 80
affine swept-wall rows with 80 partial nonlinear transition rows. It does not
append a second wall family and cannot acquire production authority.

### ShiftOut sequence 1266

- D: four solves; exact proof still rejects by about 2.278 mm;
- structured: 80 rows replace 80 rows, first solve reaches the unchanged
  4000-iteration limit in about 45 ms;
- dense: 336 additional rows, first solve also reaches 4000 iterations;
- proof-guided: four accepted corrections, then the fifth solve reaches 4000
  iterations.

Classification: structured sampling does not remove the ShiftOut numerical
failure. It is not production-ready evidence and does not prove physical
infeasibility.

### Follow sequence 531

- D: four solves; exact proof rejects by about 0.033 mm;
- structured: four solves, final solve about 35 ms, exact proof still rejects
  a later interior sample by about 0.030 mm;
- dense: 478 additional rows, one solve about 44 ms, exact proof accepts;
- proof-guided: eight solves and still rejects a moving sample.

Classification: `dense-only-interior-wall-representation-defect`. Four fixed
nonlinear samples per transition are insufficient to represent the 10 ms
physical proof, even though they use the same row count as production.

### Cruise sequence 601

- D fails numerically before proof;
- structured 80-row replacement reaches 4000 iterations in about 44 ms;
- dense 177-row augmentation reaches 4000 iterations;
- no physical artifact is produced.

Classification remains `suffix-family-unresolved`. This failure is not caused
by the Follow interior-wall mismatch alone.

## Upper-rank and primary-source check

The upper-rank log in `.steering/ano` continuously solves an `N=20`,
`dt=0.12`, `nvar=349`, `ncon=818` GMPCC while left/right candidate work runs
asynchronously. Its main solve commonly takes roughly 25--60 ms and reports
ranked candidates without the stop-and-add-cut pattern.

OSQP documents scaling as a setup-time feature and convergence as residual
balancing of an ADMM QP. The acados AS-RTI example instead keeps nonlinear OCP
structure, uses multiple-shooting integration and a structure-exploiting
partial-condensing QP solver. Those references support the next audit boundary:
compare the dense, proof-equivalent QP under an independently equilibrated
owner before considering another row topology or a new nonlinear backend.

- https://osqp.org/docs/interfaces/solver_settings.html
- https://osqp.org/docs/solver/
- https://github.com/acados/acados/blob/main/examples/acados_python/pendulum_on_cart/as_rti/as_rti_closed_loop_example.py
- https://cdn.syscop.de/publications/Diehl2005c.pdf

## Decision

Reject the fixed four-point nonlinear replacement for production. Retain it
only as an observation-only counterexample. Do not increase its sample count,
iteration limit or tolerance.

The next Slice is a solver-ownership/conditioning A/B on the already proven
dense formulation:

1. normal row-normalized OSQP (existing dense oracle);
2. a separate internally equilibrated audit owner using identical matrices,
   tolerances and warm primal;
3. unchanged exact wall, dynamic-obstacle and successor proof.

If equilibration solves ShiftOut, the defect is the numerical owner for the
proof-equivalent QP. If it does not, stop OSQP-local work and build an
independent structure-exploiting or nonlinear oracle.

## Verification

- focused structured tests: 2/2 passed;
- `make autoware-build`: 25 packages passed;
- frozen ShiftOut, Follow and Cruise replay: completed;
- full package regression: 54/54 targets, 2,149 tests, zero errors,
  failures or skips;
- `git diff --check`: passed.
