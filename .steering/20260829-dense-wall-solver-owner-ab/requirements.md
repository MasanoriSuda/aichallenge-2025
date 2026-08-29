# Requirements: dense-wall solver-owner A/B

## Objective

Separate a proof-equivalent dense wall formulation defect from the numerical
ownership/conditioning of that identical QP.

## Frozen comparison

- A: existing row-tolerance-normalized OSQP owner;
- B: separate row-tolerance-normalized owner with OSQP internal
  equilibration;
- identical dense nonlinear interior-wall rows, costs, bounds, warm primal,
  iteration limit, physical tolerance and exact proofs.

## Constraints

- Observation-only. No production authority or fallback path.
- Do not increase iteration count or loosen any tolerance or clearance.
- Do not retry B after A in one owner; each arm uses an independent workspace.
- Do not change Mission, candidate, timing or dynamic-obstacle inputs.

## Definition of done

- Solver policy is selected before solve and logged explicitly.
- Frozen ShiftOut, Follow and Cruise snapshots are evaluated by both arms.
- A/B classification is recorded before any production proposal.
- Full package build and tests pass.
