# Requirements

## Objective

Close the causal boundary between an accepted asynchronous tactical homotopy
and the live seven-state Gate A producer.

## Root cause

The tactical worker can select a feasible side and Mission geometry, but new
entry Gate A currently accepts only a same-cycle live
`V2XBehaviorState::Overtake` Mission.  Candidate generation is owned by the
worker, so the live callback remains Follow/Cruise and never submits Gate A.

## Constraints

- Do not import a worker trajectory, wall certificate or command authority.
- Accept only an async result which already passed the live result lease,
  target identity and target provenance checks.
- Pass only target/homotopy/Mission geometry to Gate A.
- Rebuild the trajectory from the current owned snapshot and the last actually
  published serialized predecessor.
- Repeat exact wall, target, successor Stop and current-world join proof.
- Do not add a timeout, lease, grace period, fallback or parameter change.
- Do not change production publisher ownership.

