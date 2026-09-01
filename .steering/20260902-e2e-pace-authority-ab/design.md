# Design

## Experiment boundary

Propagate one explicit `tiny_lidar_acceleration` launch argument from the host
environment to the existing controller parameter.  No new control owner or
fallback is introduced.

```text
TINY_LIDAR_ACCELERATION
  -> aichallenge_system.launch.xml
  -> aichallenge_submit.launch.xml
  -> reference.launch.xml
  -> control/tiny_lidar_net.launch.xml
  -> tiny_lidar_net.launch.xml
  -> node parameter acceleration
```

The controller startup log and competition analyzer bind each result to the
requested acceleration.  Old frozen evidence stays readable when no expected
acceleration is requested from the analyzer.

## Sequential Gate

1. Verify the initial `0.6 m/s2` default contract statically.
2. Run a single-vehicle `0.8 m/s2` Gate.
3. Run `1.0 m/s2` only if `0.8 m/s2` is penalty/stall free.
4. Select the fastest accepted setting, then run the deterministic NPC Gate.

This is a longitudinal-authority A/B, not a recurrent-model experiment.

The packaged default may be promoted only after both the single-vehicle and
NPC Gates accept the same value.  The rejected faster value remains available
only as an explicit experiment override.
