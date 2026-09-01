# E2E production acceptance requirements

## Objective

Evaluate the promoted TinyLidarNet checkpoint beyond the three-lap admission
world before adding another model, heuristic or threshold.  The slice covers the
six-lap practice reference first and the four-vehicle final reference second.

## Frozen production identity

- controller: `tiny_lidar_net`
- control mode: `fixed_lidar_brake`
- checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

## Constraints

- Do not tune steering, acceleration or LiDAR safety thresholds before evidence.
- Do not use GNSS, IMU, V2X, trajectory or MPC output as student features.
- Do not overwrite the production checkpoint during diagnosis.
- Keep every vehicle on the same submitted launch and checkpoint in the final
  reference world.
- Preserve generated run artifacts outside Git.

## Acceptance

1. The six-lap practice reference starts with the shipped checkpoint and
   `fixed_lidar_brake` without an override.
2. Required LiDAR, velocity and control topics are present and finite.
3. No vehicle is admitted with a post-start low-speed interval longer than 10 s
   or a positive-acceleration stall longer than 5 s.
4. AWSIM terminal state, collision evidence and per-domain motion evidence are
   recorded separately; motion distance alone is not treated as Finish proof.
5. A failed four-vehicle run is classified before implementation as perception
   generalization, lateral policy, longitudinal safety, startup/contract or
   physical contact propagation.

## Definition of Done

- Both reference targets have a reproducible command and evidence record, or the
  first failing gate has a root-cause snapshot sufficient to define one bounded
  follow-up slice.
- No production change is made without an independently rerunnable regression
  gate.
