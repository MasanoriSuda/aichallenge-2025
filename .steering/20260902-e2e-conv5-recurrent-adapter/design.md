# Design

## Frozen spatial representation

The earlier representation audit showed that the compact `fc3` feature loses
material obstacle geometry.  The new adapter therefore extracts the complete
frozen `conv5` activation, applies the same deterministic random projection
family used by the diagnostic probe, and normalizes it with statistics fitted
only from training sequences.  The projection and normalization statistics
are checkpoint buffers, not runtime configuration guesses.

The raw base steering continues through the original frozen `fc1` to `fc4`
path, then composes the exact packaged v11 spatial residual.  Both components
are immutable.  Only the recurrent correction path is trainable.  Its final
layer is initialized to exact zero so an untrained artifact emits the complete
packaged production steering exactly.

## Temporal and speed contract

The GRU consumes one projected spatial vector per LiDAR timestamp.  Wheel speed
is the already-recorded latest-preceding value; no future sample or bag
resynchronization is permitted.  Speed remains an optional, explicit model
input so the evidence can compare it without changing the ROS input contract.

## Independent-normal preservation

Normal sequences are not assigned a fictitious steering label.  Training uses
the adapter's own embedded-base output as the target and minimizes correction
to zero.  A separate normal validation objective participates in checkpoint
selection.  This avoids the previous failure mode where a candidate learned
the obstacle corpus but altered ordinary production driving.

## Authority boundary

This slice creates an offline checkpoint and gate report only.  Runtime
integration requires a separate decision after the teacher, independent-normal,
identity and bounded-output gates have been reviewed.
