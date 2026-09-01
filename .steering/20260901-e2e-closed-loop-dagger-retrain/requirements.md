# E2E closed-loop DAgger retraining requirements

## Objective

Determine whether the rejected bounded spatial-authority failure is caused by
missing closed-loop state coverage rather than by the frozen spatial
representation.

## Frozen contracts

- candidate3 and shipped runtime configuration remain unchanged;
- bounded spatial authority remains default-off;
- use the exact v2 spatial-head architecture and optimizer configuration;
- the successful authority run is train-only;
- the failed NPC authority run is held-out evaluation only;
- no threshold, authority bound or controller parameter is tuned;
- no candidate is promoted in this slice.

## Acceptance

- early stopping uses the pre-existing v3 validation split, not the newly
  observed failure;
- old and retrained candidates are evaluated on the same v4 audit split;
- report aggregate, peer, failed-prefix and last-200-sample metrics;
- production-normal leakage must remain within the existing 0.01 rad gate;
- choose retraining, temporal architecture, or data/teacher revision from
  evidence.
