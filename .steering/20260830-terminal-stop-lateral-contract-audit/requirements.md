# Requirements

## Baseline

- Source HEAD: `d6e5c8ae fix(mpcc): preserve established pass homotopy`
- Reproduced run: `output/20260830-200852`
- Frozen failure:
  `output/20260830-200852/d1/mpcc_architecture_snapshots/000000004017-ee88c9e56718aeeb-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

## Observed boundary

The persistent, stateless-left, production-left and wall-restoration primary
trajectories solve and pass exact normal wall/dynamic proof. Every one is then
rejected by the shared terminal Stop successor near waypoint 318.

The stateless maneuver declares `ContingencyStopIntent::hold_lateral_m`, but
the exact Stop builder and Emergency publisher use a zero-offset racing-line
tracking law. The declared lateral intent has no consumer.

## Objective

Determine, without changing production authority, whether the frozen failure
is:

1. genuine physical braking infeasibility; or
2. a terminal Stop candidate-generation defect in which a feasible coupled
   lateral/longitudinal braking path is excluded by a fixed lateral policy.

## Constraints

- Do not change production authority, Store mutation or publisher selection.
- Do not add a Mission rule, fallback, timeout, lease or grace period.
- Do not change solver settings, wall/vehicle clearance, braking or steering
  limits.
- Existing production Stop must remain bit-for-bit zero-offset until the audit
  proves an alternative exact trajectory and its wall/dynamic certificates.
- The audit alternative must use the same nonlinear model, command rate limits,
  wall grid, footprint, peer observation and terminal course geometry.
- Generated snapshots, logs and build output remain untracked.

## Exit classification

- zero-offset Stop fails, declared-offset Stop succeeds: lateral owner defect;
- every fixed-offset Stop fails and causal seven-state Stop succeeds: fixed
  Stop candidate-generation defect;
- fixed and seven-state Stop both fail unchanged proof: physical infeasibility
  or single-SQP limitation, classified by offline continuation;
- declared-offset exact rollout succeeds but certificate fails: model/proof
  mismatch;
- offline declared-offset succeeds but later live production fails after an
  approved promotion: scheduling or artifact-lifecycle defect.
