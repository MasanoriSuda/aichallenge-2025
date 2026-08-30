# Design

## Root cause boundary

The published normal command already owns an exact terminal Stop certificate,
but the evidence is currently consumed by a one-cycle join observer and then
discarded.  Replaying that old world trajectory is not valid: dynamic evidence
shows position differences up to roughly 0.3 m at the next control origin.

The bounded replacement is a stateless current-world successor:

1. retain the immutable identity and source plan of the last command which
   actually crossed the publisher;
2. at the current control origin, project the measured pose into the same
   course/homotopy frame;
3. rebuild the maximum-braking, path-tracking Stop suffix from that current
   state with the source seven-state physical parameters;
4. reconstruct its exact world path;
5. sweep the current wall footprint and current timed peer set;
6. report a complete shadow certificate or one structural reject reason.

This is not a second tactical branch and does not change side.  It is the
terminal successor already required by every partially proved normal Bundle,
rebuilt because state and world evidence are current rather than retained.

## Production boundary

The shadow result cannot publish, update the executed Store, alter Mission
state, or suppress Emergency Stop.  Promotion requires dynamic evidence from
an actual authority-loss decision and will replace the direct certified-
successor-to-Emergency edge atomically in a later Slice.
