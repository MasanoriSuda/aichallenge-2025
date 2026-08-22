# Track/Cruise semantic-feasibility restoration

Status: five-state-only active-set polish was dynamically falsified and
removed. This Slice records the root-cause evidence and rejected experiment;
it does not ship a production behavior change.

## Objective

Remove periodic one-cycle Emergency authority caused by an OSQP `solved`
result whose executable five-state box row is outside its own semantic
tolerance, without weakening the canonical execution contract or changing the
legacy solver contract.

## Baseline evidence

- Six-lap run: `output/20260823-075629`
- Observer-enabled control: `output/20260823-081219`
- Rejected shared rowwise admission: `output/20260822-234326`
- Rejected whole-QP row scaling: `output/20260823-063519`

In the six-lap run there are 97 typed `execution-primal-reject` outcomes:

- curvature stage 0: 81 (75 warm, 6 cold)
- acceleration: 10 (7 at stage 0, 2 at stage 1, 1 at stage 6)
- virtual-progress speed stage 0: 4
- predicted velocity stage 1: 2

## Constraints

- Do not widen a physical input/state bound or semantic tolerance.
- Do not clamp an uncertified primal and publish it under the old certificate.
- Do not change the shared legacy solver behavior.
- Do not add a runtime flag, retry loop, timeout, compatibility fallback or
  legacy authority path.
- Do not tune MPC/MPCC weights, wall margin, speed or steering limits.
- Preserve raw solver primal/dual for warm start and residual provenance.
- Preserve ROS interfaces and the canonical Fresh/Retained/Emergency authority
  graph.

## Acceptance

- The existing dynamic 97-event baseline remains the failure reproduction. A
  small synthetic mixed-unit QP is not treated as equivalent unless it actually
  reproduces the local-row miss.
- The selected five-state-only correction produces an executable primal inside
  every semantic box row or fails closed.
- Complete package tests pass.
- A one-car dynamic run has zero `execution-primal-reject`, no new solver
  failure cascade and no continuous callback overrun.
