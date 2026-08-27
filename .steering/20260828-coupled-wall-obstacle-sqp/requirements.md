# Requirements

## Objective

Remove the structural infeasibility created when dynamic-obstacle rows are
added after progress-aligned wall refinement has already frozen progress to a
wall-profile segment.

## Frozen evidence

- Run: `output/20260828-025409`
- Snapshot:
  `d1/mpcc_architecture_snapshots/000000002142-pass-dynamic-obstacle-refinement-solve-rejected/snapshot.yaml`
- Warm and cold replay reproduce the same stage-16 effective-progress row.
- The complete affine problem is infeasible.
- Removing only the 16 dynamic-obstacle rows makes the affine problem
  feasible.
- From stage 6 onward, the wall-refined progress box is already ahead of the
  obstacle's stay-behind upper bound. At stage 16 the minimum effective
  progress is `15.85 m`, while the obstacle upper bound is `12.091 m`.

## Constraints

- Do not change solver tolerances, wall/vehicle clearance, authority,
  fallback, timeout, lease, or grace periods.
- The wall-only solution may select the obstacle homotopy, but its narrow
  progress trust bucket may not prevent the coupled problem from braking.
- The final published artifact must be solved with both dynamic-obstacle and
  physical wall constraints present.
- Preserve the exact physical proof as the publication Gate.

## Definition of done

- Dynamic-obstacle rows can be generated from one physical witness and
  applied to a different, compatible broad problem.
- With both refinements active, the solver performs a bounded coupled
  sequence: wall witness, dynamic-aware solve, wall refinement around the
  dynamic-aware trajectory, joint solve.
- Focused regression tests prove that witness progress boxes are not copied
  into the dynamic-aware problem.
- Package build/tests pass and a dynamic Gate no longer reproduces the frozen
  wall-progress/dynamic-progress contradiction.
