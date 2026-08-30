# Requirements

## Objective

Close the evidence gap identified by the frozen Pass artifact-expiry audit
without granting production authority.  Determine whether the terminal Stop
successor of the last actually published ManeuverBundle can be rebuilt from
the current control-origin state and certified against the current wall and
all current dynamic obstacles.

## Baseline

- `3b7b8592 audit(mpcc): classify pass artifact expiry`
- Frozen failure: `output/20260830-162637`, decision 5698

## Constraints

- Production authority and ROS output are unchanged.
- No Mission resume rule, lease, grace, timeout, retry, solver tolerance,
  clearance, or configuration parameter is added.
- Do not replay the historical terminal trajectory as if the measured state
  matched it.  The successor must start at the current control origin.
- Preserve the published source solution, target, intent and homotopy
  identity.
- Rebuild with the same physical model, maximum-braking envelope and Stop
  lateral policy used by the existing terminal certificate.
- Require current static-wall and all-peer timed-path proof.

## Exit criteria

- A deterministic test proves that an exhausted source prefix can still form
  a current-world Stop successor without extending the prefix cursor.
- Invalid identity, current state, wall path and dynamic path fail closed.
- Shadow telemetry distinguishes unavailable, wall-blocked, dynamic-blocked
  and certified results.
- A dynamic run shows whether the successor would have covered an actual
  normal-authority loss.  Production promotion is a separate Slice.
