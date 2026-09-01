# Evidence

## Generated corrective source

`dataset/competition_failure_teacher_v1` is generated and excluded from git.

| split | source | accepted | material | causal cutoff |
|---|---|---:|---:|---:|
| train | NPC `20260901-152109/d1` | 5710 | 847 | confirmed contact, 1 s margin |
| train | peer `20260901-153143/d1` | 722 | 534 | confirmed contact, 1 s margin |
| val | peer `20260901-153143/d3` | 452 | 16 | confirmed contact, 1 s margin |

Every sequence identifies `LidarPrecontactTeacher`, frozen candidate3 and SHA
`de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.

## Combined temporal dataset

- generated root: `dataset/recurrent_direct_v3`
- schema: v2, physical scan unit metres
- sources: `precontact_residual_base_v4` plus
  `competition_failure_teacher_v1`
- train: 9 sequences, 29304 samples
- validation: 4 sequences, 12313 samples
- total: 13 sequences, 41617 samples
- maximum odometry synchronization delta: 47.305 ms
- physical scan range: 0.4993--30.0 m
- train/validation identity overlap: none

The minimum range confirms that the 0.5 m contact suffix is not present.  The
next offline probe must compare static compact features with temporal full
geometry on this exact split.  It is not permitted to train on peer d3 or move
that sequence into train after seeing its result.

## Verification

```text
focused recurrent contract tests     14 passed
TinyLidarNet full tests              102 passed
```
