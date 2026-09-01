# Design

Run two sequential `e2e-npc-single` sessions with only
`E2E_START_RANDOM_SEED` changed.  Runtime launch provenance must report the
packaged `0.8 m/s2` acceleration without `TINY_LIDAR_ACCELERATION` being set.

For each run:

1. wait for the complete AWSIM result rather than judging motion only;
2. run the stall analyzer on the MCAP;
3. run the strict competition analyzer with expected runtime identity;
4. record lap, penalty, speed and safety-intervention evidence.

No implementation authority changes in this slice.

If an unseen seed rejects `0.8 m/s2`, rerun that exact seed with the previous
`0.6 m/s2` authority.  A passing control run classifies the higher fixed
acceleration as a closed-loop distribution regression.  Do not hide that
regression by selecting a nearby constant; restore the admitted baseline and
separate transient acceleration from steady-state speed in a later Slice.
