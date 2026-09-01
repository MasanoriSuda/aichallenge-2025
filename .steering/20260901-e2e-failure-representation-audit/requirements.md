# E2E failure representation audit requirements

## Objective

Determine whether the rejected spatial-authority wall failure can be learned
from the current frozen candidate3 representation, or requires raw LiDAR and
short observation history.

## Frozen contracts

- candidate3 and the shipped runtime configuration do not change;
- the spatial-authority runtime remains default-off;
- no authority bound, clearance or longitudinal threshold is tuned;
- the successful authority run is train-only;
- the failed NPC authority run is validation-only and may not be moved to
  training after inspecting its result;
- no probe creates a promotable runtime checkpoint.

## Compared representations

1. compact candidate3 feature plus speed;
2. frozen candidate3 conv5 spatial feature plus speed;
3. frozen conv5 feature plus 1/8-frame differences and speed differences;
4. normalized raw 750-point LiDAR plus speed;
5. normalized raw LiDAR plus 1/8-frame differences and speed differences.

## Acceptance

- immutable run-level identities and train/validation separation;
- physical metre and synchronized speed contracts remain valid;
- the final 10 seconds before the validation wall failure are reported
  separately from aggregate validation;
- choose a next architecture from evidence; do not promote a runtime model in
  this slice.
