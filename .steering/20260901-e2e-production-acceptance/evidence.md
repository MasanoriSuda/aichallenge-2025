# Evidence

## Six-lap practice reference

- command: `make e2e`
- run: `output/20260901-084641`
- controller: shipped `tiny_lidar_net`, `fixed_lidar_brake`
- checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- AWSIM reached `Finish` and finalized the bag after about 593 s.

| distance | bag duration | max speed | longest low speed | positive-accel stall | result |
|---:|---:|---:|---:|---:|---|
| 2018.59 m | 584.09 s | 4.57 m/s | 1.57 s | 0.54 s | pass |

The longitudinal safety layer inhibited acceleration during one close encounter
and then returned to `clear`; live velocity recovered to 4.13 m/s.

## Four-vehicle final reference

- command: `make e2e-final`, then `make awsim-request-start` after all four
  domains reported `Grounded`
- run: `output/20260901-085903`
- world: four vehicles, no NPC, six laps, collisions on, wall recovery off
- all four domains loaded the same shipped checkpoint and
  `fixed_lidar_brake`
- the run was stopped after the admission failure was stable and all four bags
  were finalized

| domain | distance | duration | longest low speed | positive-accel stall | result |
|---|---:|---:|---:|---:|---|
| d1 | 735.76 m | 306.91 s | 0.00 s | 0.00 s | pass |
| d2 | 111.25 m | 315.46 s | 100.47 s | 61.56 s | fail |
| d3 | 549.64 m | 324.17 s | 62.57 s | 1.73 s | fail; recovered |
| d4 | 108.03 m | 332.96 s | 117.73 s | 79.28 s | fail |

At the start of the persistent positive-acceleration stall, d2 and d4 were at
approximately `(89641.12, 43137.80)` and `(89641.20, 43139.34)`, only 1.54 m
apart.  At the end they remained about 1.50 m apart.  Both commanded
`+0.6 m/s2`; their front sectors were open at 7--8 m, while the closest returns
were lateral.  This is not an intentional front-clearance stop or a startup
contract failure.  It is a side-contact/physical embedding state created before
the longitudinal policy returned to clear.

d3 entered a different close-range state (`front=1.53 m`, closest side return
`0.49 m`) and remained slow for 62.57 s, but subsequently escaped and reached
549.64 m.  This shows that the policy can sometimes recover, but not reliably.

## Classification

The first failing cause is **lateral-policy generalization under symmetric peer
interaction, followed by physical contact propagation**.  Changing the LiDAR
brake threshold cannot resolve a vehicle that is already side-pinned with an
open front sector.  A heuristic recovery owner would also violate the current
ML lateral-authority objective.

The bounded next experiment is an unchanged-world A/B in which only d4 uses the
existing teacher-only `gap_teacher`; d1--d3 retain production.  If d4 avoids the
contact and passes the stall gate, the next data slice may collect pre-contact
teacher labels.  If it fails in the same geometry, another training run is not
justified and the teacher/candidate representation must be reconsidered.
