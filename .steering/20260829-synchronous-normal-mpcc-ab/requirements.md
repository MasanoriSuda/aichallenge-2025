# Requirements

## Objective

Test whether the remaining canonical normal-authority loss is created by the
asynchronous candidate-adoption lifecycle.  Compare the failed retained join
with a same-cycle solve of the identical seven-state problem, exact wall proof
and current-world proof.

## Frozen evidence

- Baseline: `f7012260` plus the isolated late-ShiftOut candidate change.
- Failure run: `output/20260829-001005/d1`, decision 979.
- Upper-rank reference: `.steering/ano` runs the main GMPCC solve directly and
  moves only tactical alternatives to a child process.

## Constraints

- Observation-only A/B first; production authority remains unchanged.
- Use the identical seven-state formulation, physical wall proof, dynamic
  obstacle proof and publication reachability proof.
- Bind the solve to the last command which actually crossed the wire.
- Do not change solver settings, clearances, cadence parameters, timeouts,
  leases, grace periods or fallback behavior.
- If the synchronous arm succeeds, it must replace the defective asynchronous
  normal-production lifecycle before closure; both paths may not remain as
  permanent production owners.

## Exit criteria

- At a frozen async `steering-unreachable` authority hole, record whether the
  same-cycle arm solves, passes physical proof and produces current-world
  authority.
- If both fail for the same physical reason, reject the scheduling hypothesis.
- If async fails and synchronous succeeds, classify the defect as normal
  producer scheduling/lifecycle and design the removal of async normal
  adoption.
- Full build and package tests pass.
