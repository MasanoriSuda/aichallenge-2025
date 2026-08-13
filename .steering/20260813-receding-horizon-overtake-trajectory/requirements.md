# Requirements

## Objective

Replace the active overtake line's single lateral-goal horizon with a bounded
receding-horizon lateral trajectory optimization.  The low-level 40 Hz MPC,
hard wall checks, target-footprint policy, and existing Mission lifecycle stay
in place.

## Scope

- Optimize every active ShiftOut/Pass horizon as multiple lateral samples.
- Keep the selected pass-side topology while ego and the target can overlap.
- Allow the future samples to respond to changing wall bounds and course
  curvature instead of holding one Frenet offset for the whole Pass.
- Warm-start from the previous control-cycle solution to suppress chatter.
- Fall back to the existing Mission horizon when the continuous problem is
  invalid or infeasible.
- Add deterministic unit tests and runtime diagnostics.

## Constraints

- Do not change ROS topic, message, service, launch, or evaluation contracts.
- Do not weaken physical wall-contact handling or Recovery terminal guards.
- Do not claim full MPCC: this is the first receding-horizon lateral-planner
  stage, feeding the existing MPC controller.
- Preserve the user's existing `aichallenge/result-summary.json` modification.

## Definition of done

- The optimizer returns a bounded, smooth multi-sample lateral trajectory.
- Target-side separation is a hard bound over the predicted overlap window.
- Invalid bounds produce a clean fallback, not a partially optimized horizon.
- Package unit tests and `make autoware-build` pass.
