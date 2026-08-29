# Requirements: wall-refinement equilibrated production owner

## Objective

Promote the numerically validated internally equilibrated OSQP policy as the
single production owner of wall-refinement and coupled wall/opponent QPs.
Initial, successive-linearization and dynamic-only QPs keep their existing
canonical owner because the frozen corpus proves that one global scaling
policy is not valid for every problem class.

## Root-cause evidence

- Frozen ShiftOut wall fingerprint `9845010060330222052` is physically
  feasible and passes every unchanged exact proof, but the current wall solve
  reaches OSQP's iteration limit.
- Frozen coupled wall/opponent fingerprint `7896913873338064473` also solves
  and passes the unchanged proof chain with internal equilibration.
- Frozen dynamic-only fingerprint `5862539731343104692` remains invalid under
  that policy, so the policy must not be promoted globally.
- Explicit fixed-pass Ruiz variants regress the known Follow sequence 5575 and
  therefore are not a canonical global replacement.

## Constraints

- Do not add a retry, fallback, lease, timeout, grace period, tolerance or
  clearance change.
- A submitted QP has exactly one solver owner selected from its immutable
  problem class before solving.
- Do not change candidate geometry, physical constraints, objective, warm
  start, exact wall proof or publication authority.
- A numerical solve is still insufficient: the existing affine, nonlinear
  wall, timed-obstacle and terminal-successor proofs remain authoritative.
- Preserve the ROS 2 and evaluation interfaces.

## Definition of done

1. Wall-refinement and coupled wall/opponent solve sites use the equilibrated
   owner and cannot reach the old owner.
2. Initial, successive-linearization, dynamic-only and latest-state feedback
   solve sites retain the existing owner.
3. Source-contract tests prove the owner partition and absence of retry logic.
4. Both frozen wall fingerprints solve and pass the unchanged exact proof
   chain; the dynamic-only counterexample remains rejected.
5. Package tests, workspace build and `make dev2` dynamic acceptance complete.
