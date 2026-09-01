# Design

Run `e2e-npc-single` with `TINY_LIDAR_CONTROL_MODE=precontact_teacher` and an
unseen seed.  This keeps the LiDAR-only input contract and the two dynamic NPCs
while making the label-producing policy the actual steering owner.

The gate is run-level rather than frame-level.  Teacher actions become hard
demonstrations only if their consequences complete all three laps without a
penalty or stall.  No portion of a failed run is promoted merely because an
individual steering command looks plausible.

## Result

The exact teacher passed unseen seed 2031:

- laps: `103.9225 / 90.2950 / 99.1202 s`;
- Finish: 3/3 laps, first place;
- penalty: 0;
- distance: 1,030.188 m;
- post-start low-speed and positive-acceleration stall: 0 s;
- LiDAR samples: 6,244, minimum front range 2.000 m;
- runtime: `precontact_teacher`;
- frozen base SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.

This is the first strict `executed_teacher_success` source available to the
current label pipeline.  It may be extracted only after its result and motion
artifact identities are embedded in the derived dataset metadata.
