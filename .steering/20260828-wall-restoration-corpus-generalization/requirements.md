# Requirements

## Objective

Generalize the audit-only wall feasibility restoration from the decision-2473
heading witness to the actual refinement-owned pose-bucket boundary, then
replay the frozen wall-failure corpus without changing production authority.

## Frozen evidence

- 17 exact failure QPs were checked independently with HiGHS LP.
- 13 are affine-infeasible and four are affine-feasible.
- decision 2473 is restored by relaxing the refinement heading bucket.
- decision 1566 remains infeasible under heading-only restoration but becomes
  feasible when all refinement-owned pose buckets are removed.
- decision 1161 remains infeasible until the complete physical wall envelope
  is removed; it is not a restoration candidate.

## Constraints

- Do not change production `evaluate()` behavior or authority.
- Do not change clearance, wall interval, actuator limits, OSQP settings,
  Mission lifecycle, timeout, lease, grace or fallback.
- Pose-bucket relaxation is Phase-I seed generation only. Explicit progress
  wall and swept-footprint rows remain present.
- Rebuild a fresh full physical wall problem before any artifact can exist.
- Require exact QP, nonlinear trajectory, wall, dynamic and successor proofs.

## Definition of Done

- Restoration ownership matches lateral/lag/heading/progress buckets created
  by physical wall refinement rather than one frozen incident.
- decision 2473 remains classified correctly.
- decision 1566 and the remaining wall corpus are replayed and classified.
- Production recommendation distinguishes construction, solver and physical
  infeasibility instead of proposing one universal fallback.
