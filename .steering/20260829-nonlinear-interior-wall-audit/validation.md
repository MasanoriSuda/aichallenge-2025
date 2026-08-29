# Validation

## Frozen comparison

All arms use the same immutable snapshots, x0 probe, costs, endpoint boxes,
wall intervals, clearances, solver settings and exact physical proof. E only
adds linearized true-nonlinear lateral rows at the proof's interior 10 ms
samples. It is not connected to production authority.

### Follow sequence 531

- D: four reachable SQP solves, then exact proof rejects dense stage 473;
- D lateral violation: about 0.033 mm with 0.010 mm tolerance;
- E: 478 additive rows, one solve, unchanged exact proof accepts;
- classification: `nonlinear-interior-wall-representation-defect`.

This is direct evidence that endpoint boxes plus affine endpoint-interpolation
rows were not equivalent to the exact nonlinear interior-wall certificate.
No tolerance or clearance change was needed.

### ShiftOut sequence 1266

- D: four reachable SQP solves, then exact proof rejects dense stage 339;
- D lateral violation: about 2.278 mm with 0.010 mm tolerance;
- E: 336 additive rows; the first QP reaches the unchanged 4000-iteration
  limit (`status=-2`) without an accepted solve;
- the initial reachable candidate itself does not violate E's tangent rows;
- classification remains `solve-proof-model-mismatch` because E does not
  produce an artifact.

E therefore catches a material representation boundary but does not establish
that this frozen ShiftOut suffix is feasible for the unchanged live solver.
It must not be promoted by adding iterations or relaxing proof thresholds.

### Cruise sequence 601

- A/B/C/D already fail numerically;
- E adds 177 rows and also reaches the unchanged 4000-iteration limit;
- classification remains `suffix-family-unresolved`.

This is not an interior-wall proof case: no arm produces a physical artifact.
It remains a separate candidate/conditioning/physical-feasibility audit.

## Root-cause conclusion

The frozen failures are not one defect. Follow proves a genuine wall
representation mismatch. ShiftOut has the same downstream proof symptom under
D, but a naive dense-row formulation is not accepted by the current solver.
Cruise fails before proof. Treating all three by another grace period,
tolerance, wall margin or Mission resume rule would hide distinct causes.

The production implication is deliberately limited: exact interior-wall
semantics must eventually enter candidate construction or the live
optimization certificate, but the full dense E formulation is an audit oracle,
not a production design. A production form needs sparse/selected constraints
or another efficient certificate and independent timing evidence.

## Verification

- focused prefix-preservation test: passed;
- focused observation-only exact-proof test: passed;
- immutable replay of Follow, ShiftOut and Cruise: completed;
- `make autoware-build`: 25 packages passed;
- `colcon test --packages-select multi_purpose_mpc_ros`: 54/54 test
  targets passed;
- `git diff --check`: passed.
