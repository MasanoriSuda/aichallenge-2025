# Design

`can_release_overtake_front_cap()` distinguishes two constrained-horizon paths:

1. Initial release: requires `lateral_separation_clear`, which represents the full configured
   release threshold.
2. Existing-release hold: requires the existing release state and current separation above the
   lower reapply threshold.

The controller allows these paths only when phase is `Pass`, physical front-overlap exclusion is
latched, the generated path remains physically feasible, and the actual footprint is not in wall
contact.

This deliberately ignores only reference-level `wall_limited`, `static_wall_limited`, and
`lateral_accel_limited` flags. It does not ignore physical infeasibility or any hard longitudinal
safety bound.

