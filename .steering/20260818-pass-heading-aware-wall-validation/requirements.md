# Requirements

## Purpose

Reduce Pass/Return interruptions caused by a lateral path that is accepted by
the DP/receding-horizon wall check but rejected by the MPCC execution wall
check once the kart heading induced by `d(s)` is considered.

## Evidence

- Run `output/20260818-003303` completed 77 P1 laps.
- Pass completion improved, but 65 clustered MPCC hard-wall releases remained.
- 108/184 hard-wall log samples occurred around `wp_id=313..33` across the
  circular trajectory boundary.
- The planning wall clamp samples the footprint at the base-path heading,
  while solved MPCC authority checks include the lateral-profile heading.

## Scope

- Participant package only:
  `aichallenge_submit/multi_purpose_mpc_ros`.
- Preserve all ROS 2 topic, service, launch and evaluation contracts.
- Preserve the configured hard wall clearance and physical kart footprint.
- Do not change tuning parameters or the user's local `config.yaml` change.

## Acceptance criteria

- Wall clearance search can evaluate an explicit path-heading offset while
  retaining Frenet lateral translation relative to the base path.
- Overtake horizon validation uses the same `d(s)` heading convention as MPCC
  solved-trajectory authority validation.
- A wall-side target may be moved monotonically toward the base line; no path
  may be accepted by weakening physical collision or configured hard margin.
- Unit tests, package tests and `make autoware-build` pass.
