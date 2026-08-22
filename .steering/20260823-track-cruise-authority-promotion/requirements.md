# Track/Cruise canonical authority promotion

## Purpose

Promote the already certified five-state canonical MPCC chain to the sole normal authority for
Track/Cruise.  This Slice is the explicit Slice 3 authority boundary.  The design/audit portion does
not change production commands; implementation begins only after explicit approval.

## Required normal chain

```text
fresh certified canonical plan
-> current-world revalidated retained canonical plan
-> Emergency Stop
```

After promotion, Track/Cruise must not solve or publish the three-state/legacy MPC as a cycle-local
fallback.

## Constraints

- Keep `/control/command/control_cmd` and all launch/topic/message contracts unchanged.
- Recovery may override normal authority, but it must remain explicitly identifiable as Recovery.
- Preserve the canonical plan/problem/decision identity through final command publication.
- Preserve the exact optimized acceleration and steering carried by canonical actuation unless an
  explicit Emergency or Recovery override owns the command.
- Do not add a feature flag, timeout, lease, retry, circuit breaker, tuning value or compatibility
  fallback.
- Do not tune OSQP, wall margin, cost weights, speed limits or controller gains in this Slice.
- Delete the replaced Track/Cruise legacy normal branch in the same authority-changing commit.
- Follow/Hold/Stop/Overtake remain outside this Slice and continue through their current paths.
- Preserve the user-owned `aichallenge/result-summary.json` change.

## Acceptance

- Failure-first tests prove the old `{speed, steering}` return contract cannot represent canonical
  acceleration and source identity.
- A pure final-normal-command resolver selects only FreshCertified, RetainedCertified or
  EmergencyStop for Track/Cruise.
- Fresh/retained canonical actuation reaches the final command without normal post-processing that
  invalidates the certified lateral command.
- Safety and Recovery overrides remain explicit and dominate normal authority.
- Static search finds no Track/Cruise cycle-local call to legacy/three-state solve after promotion.
- Every published normal Track/Cruise command names decision, plan, formulation and source.
- Build, full package tests and repeated dynamic runs pass with zero formulation switches.

## Approval boundary

Connecting this resolver to `publish_control_command()` and deleting the Track/Cruise legacy branch
changes the vehicle's production command authority.  That implementation is intentionally pending
explicit user approval.
