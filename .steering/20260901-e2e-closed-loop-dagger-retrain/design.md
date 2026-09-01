# Design

Create an ignored training view containing the existing recurrent v3 corpus
plus only the successful authority sequence.  Its validation split remains
byte-identical to recurrent v3, so the failed run cannot influence gradient
updates or early stopping.

Train one candidate with the v2 configuration:

- frozen candidate3 backbone;
- projected conv5 spatial feature, 128 dimensions;
- synchronized speed;
- fixed train statistics;
- signed-mixture head;
- sample-balanced training.

Evaluate both v2 and the new candidate on recurrent v4, which adds the held-out
failed NPC prefix.  `evaluate_spatial_adapter.py` reports the focused run and a
causal last-200-sample suffix so aggregate performance cannot conceal the wall
failure state.

The evaluator also applies the exact runtime authority clip (currently
plus/minus 0.12 rad) before reporting a second set of residual metrics.  The
unbounded 1.2 rad model output is useful for representation analysis but is not
the command that the controller can publish, so it cannot be the sole offline
acceptance evidence.

This is an offline architecture/data audit.  It does not change ROS launch,
parameters, runtime weights, or authority.
