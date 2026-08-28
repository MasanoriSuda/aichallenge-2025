# Requirements

## Objective

Determine why frozen Follow snapshot 5575 is mathematically feasible as an
exact affine QP but the unchanged production OSQP reaches 4000 iterations in
both warm and cold replay.

## Evidence boundary

- Snapshot:
  `output/20260828-171709/d1/mpcc_architecture_snapshots/000000005575-follow-wall-refinement-coupled-solve-rejected/snapshot.yaml`
- HiGHS previously found a feasible point for the exact physical-coordinate
  rows.
- Production OSQP did not converge with either the recorded warm start or a
  cold start.

## Constraints

- Do not change production solver tolerances, iteration count, clearance,
  actuator limits, velocity limits, authority, fallback, lease or timeout.
- Separate mathematical feasibility from numerical conditioning.
- Inspect the exact transformed problem used by OSQP: variable coordinates,
  tolerance-normalized rows, objective scale and active-row dependence.
- Any correction must name the earliest violated numerical/dataflow invariant
  and remove or unify its cause rather than special-case snapshot 5575.

## Definition of done

- Warm and cold nonconvergence are reproduced from the frozen payload.
- Exact-QP feasibility is independently reconfirmed.
- Dominant scale/dependence/KKT defect is identified or explicitly falsified.
- A production change is made only if it follows one-to-one from that defect;
  otherwise the Slice ends with a bounded external comparison question.
