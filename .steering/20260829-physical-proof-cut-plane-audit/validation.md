# Validation

## Frozen replay

All comparisons retain the same snapshot, latest-state probe, costs, physical
bounds, solver settings and exact proof. F has no production authority.

### ShiftOut sequence 1266

- D: four solves, exact proof still rejects dense stage 339 by 2.278 mm;
- E: 336 rows, first solve reaches 4000 iterations;
- F: four proof-selected cuts and four accepted solves;
- the rejected sample moves as cuts are added;
- attempt five reaches the unchanged 4000-iteration limit with four rows;
- the last exact evidence before solver rejection is dense stage 342, with a
  1.344 mm violation.

The dense-row failure is not explained only by row count: even four selected
rows eventually reach the live iteration ceiling after the nonlinear
violation moves. This frozen suffix remains unresolved and is not promoted.

### Follow sequence 531

- E: 478 simultaneous rows, one solve in about 44 ms, exact proof accepts;
- F: seven cuts and eight accepted solves, but the rejection moves to dense
  stage 480;
- the final violation is about 0.075 mm with a 0.010 mm tolerance;
- the final solve takes about 37 ms, excluding the seven preceding solves.

This falsifies the hypothesis that a one-point-at-a-time cut-plane is an
efficient production replacement for the dense wall representation. It is a
literal numerical form of the prior symptom-patching loop: each local cut
moves the exact-proof failure to a neighboring sample.

### Cruise sequence 601

- the base reachable problem reaches 4000 iterations before physical proof;
- F has zero cuts because there is no exact wall rejection sample;
- classification remains `suffix-family-unresolved`.

Cruise is therefore independent of the interior-wall certificate problem.

## Conclusion

The evidence supports a structured, horizon-wide interior wall representation
or a true nonlinear feasibility/refinement method, not incremental Mission
rules and not single-sample proof patches. Dense E is a useful offline oracle
for Follow but is too slow and does not solve ShiftOut. F is retained only as
an architecture counterexample and provenance test.

No production implementation should adopt E or F without a separate sparse
formulation and timing Slice. Parameter tuning cannot resolve this mismatch.

## Verification

- six focused proof/mapping/prefix tests: passed;
- immutable ShiftOut, Follow and Cruise replay: completed;
- `make autoware-build`: 25 packages passed;
- `colcon test --packages-select multi_purpose_mpc_ros`: 54/54 test
  targets passed;
- `git diff --check`: passed.
