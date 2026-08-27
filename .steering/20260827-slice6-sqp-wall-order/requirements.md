# Requirements

## Objective

Restore a feasible canonical seven-state solve without relaxing any wall,
dynamic-obstacle, actuator or solver contract.

## Evidence

- `output/20260827-172718`: canonical normal authority remained Emergency.
- `output/20260827-175049`: nonlinear tangent construction succeeded after
  exact-box projection, but relinearized QPs remained infeasible.
- The wall refinement owned narrow progress/lag/heading buckets before all
  temporal dynamics rows were replaced.

## Constraints

- Do not tune OSQP, wall clearance, horizon duration or fallback timing.
- Do not clamp a solved command or weaken physical certification.
- Keep one canonical seven-state formulation and exact physical replay.
- Preserve ROS 2 and evaluation interfaces.

## Definition of Done

- The nonlinear tangent is selected from a physically valid box point.
- The relinearized QP is initialized from its own equality system.
- Wall and dynamic-obstacle refinements are built after relinearization.
- Full build/tests pass.
- A moving `make dev2` run demonstrates canonical Track/Cruise, Follow and
  ShiftOut authority.
