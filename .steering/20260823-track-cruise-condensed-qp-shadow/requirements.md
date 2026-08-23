# Track/Cruise condensed QP shadow

## Objective

Test the structural hypothesis that the canonical five-state QP is unreliable
because state variables and dynamics equalities are solved only approximately.
Build an exactly equivalent control-input-only QP, reconstruct every state by
the declared linearized dynamics, and compare it in runtime shadow without
changing command authority.

## Root-cause evidence

- `output/20260823-075629` contained repeated small-unit input/state row
  rejects after OSQP reported solved.
- The rejected primal restoration in `output/20260823-090945` changed 120--145
  fields when exact dynamics were rolled from measured state zero. The raw
  horizon was therefore not merely one clipped actuator value away from its
  declared dynamics.
- Row scaling, dual rebasing, polish, downstream restoration, physical
  progress coupling, scaled termination and full nondimensionalization were
  all dynamically falsified and removed.

## Constraints

- Preserve the exact physical objective, state/input/rate bounds and dynamics.
- Eliminate state variables and dynamics equality rows algebraically; do not
  approximate or tune them.
- Reconstruct the full five-state primal before using existing semantic
  diagnostics.
- Keep the current expanded canonical solver as the only production authority
  in this Slice.
- The condensed solve is observation-only and cannot enter the plan store,
  retained selector or publisher.
- Add no parameter, flag, retry, timeout, fallback, clamp, margin or gain.
- Do not change ROS, launch, V2X, wall or Recovery contracts.

## Acceptance

- Failure-first tests show the existing API cannot express an affine condensed
  problem or reconstruct an exact full primal.
- Pure tests prove objective and constraint equivalence for arbitrary control
  vectors and exact dynamics reconstruction.
- Malformed canonical layouts fail closed.
- Focused, complete package and build tests pass.
- A short shadow run has no callback overrun attributable to the observer.
- Condensed shadow has zero expanded dynamics residual and materially fewer
  semantic execution rejects than the production expanded solver.
- First actuation differences are measured, not silently adopted.

This Slice stops after shadow evidence. Production replacement and deletion of
the expanded Track/Cruise solver require a separate accepted Slice.
