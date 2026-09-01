# E2E DAgger v3 NPC repeat requirements

## Objective

Confirm that the v3 NPC-authority success is reproducible under a different
admitted randomized start placement.

## Frozen inputs

- candidate and base checkpoint identities are unchanged;
- control mode and `0.12 rad` authority bound are unchanged;
- runtime NPC scenario is unchanged;
- only `E2E_START_RANDOM_SEED` changes from `2026` to `2027`;
- production defaults remain spatial authority OFF.

## Acceptance

- ego Finish 3/3, penalty zero and stall zero;
- spatial runtime gate passes;
- no v2-style wall lock;
- completion is reproduced across both seeds.
