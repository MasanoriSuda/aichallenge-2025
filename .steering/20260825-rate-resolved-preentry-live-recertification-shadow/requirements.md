# Requirements

## Objective

Determine whether the selected Overtake homotopy can be rebuilt and certified
from the live committed vehicle state, instead of attempting to adopt the
immutable six-state trajectory produced by an older asynchronous tactical
snapshot.

## Root-cause evidence entering this Slice

- The asynchronous left/right worker produces valid solver, wall and target
  certificates.
- Live retained adoption rejects those plans because Track/Follow continues to
  change steering and speed while the tactical result is in flight.
- Advancing the artifact cursor does not repair that causal mismatch; the
  current committed predecessor is no longer the predecessor used by the old
  plan.

## Constraints

- Observation only. Do not grant production authority to the new result.
- Do not relax steering, velocity, wall or target thresholds.
- Do not add a fallback, timeout, lease or configuration flag.
- Use the selected asynchronous result only as a homotopy hint.
- Rebuild from a current model/V2X snapshot and the current committed steering.
- Evaluate at most once per selected asynchronous plan identity.
- Preserve the existing Emergency/Recovery override and ROS interfaces.
- Do not stage or modify `aichallenge/result-summary.json`.

## Acceptance

- Logs distinguish old-artifact adoption from current-state recertification.
- Current-state recertification reports selected side, current candidate
  availability, six-state solver/wall/target result and total callback cost.
- The recertified plan cannot reach Mission commit or command publication.
- Build and focused/full package tests pass.
- A bounded `make dev2` run establishes success rate and worst-case cost.
