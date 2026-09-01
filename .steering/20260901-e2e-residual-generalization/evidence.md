# Evidence

## Run-level teacher admission

Both runs used the frozen production TinyLidarNet base with diagnostic-only
`precontact_teacher` lateral authority and two runtime NPCs.

| seed | run | finish | laps | total lap time | distance | mean speed | stall |
|---:|---|---|---:|---:|---:|---:|---:|
| 2027 | `output/20260901-125752` | yes | 3 | 299.78 s | 1024.31 m | 3.22 m/s | 0 s |
| 2028 | `output/20260901-130837` | yes | 3 | 289.87 s | 1030.13 m | 3.34 m/s | 0 s |

The analyzer admitted both bags.  Seed 2027 reached a front LiDAR minimum of
0.75 m; seed 2028 reached 2.85 m.  They therefore add materially different
dynamic-obstacle proximity while remaining completed, non-stalled runs.

## Seed-disjoint dataset

Generated dataset (not committed):

```text
aichallenge/ml_workspace/tiny_lidar_net/dataset/precontact_residual_base_v4
```

- seed 2027 was added only to train: 6372 samples, 618 material;
- seed 2028 was added only to validation: 6173 samples, 556 material;
- the frozen base checkpoint SHA remained
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.

## Frozen architecture result

Generated checkpoint directory (not committed):

```text
aichallenge/ml_workspace/tiny_lidar_net/checkpoints/residual-temporal-v4/20260901_131646
```

The validation-selected checkpoint remained the exact zero residual.  The
final optimizer state produced the following diagnostic metrics:

| subset | material MAE improvement | anchor MAE |
|---|---:|---:|
| new d1 final 10 s | 54.8% | 0.0121 rad |
| new d2 final 10 s | 58.1% | N/A |
| seed 2027 full train run | 0.7% | 0.0741 rad |
| unseen seed 2028 full run | -2.9% | 0.0761 rad |
| full validation | 9.8% | 0.0730 rad |

Independent normal leakage was 0.0858 rad against the 0.01 rad limit.

## Classification

Adding independent successful trajectories does not make the current binary
gate/two-head residual generalize.  The model memorizes the two strongly signed
failure tails, but it does not learn even the full successful teacher run in
its training split and worsens the unseen seed.

The remaining target is multi-modal: left correction, no correction and right
correction are qualitatively different actions.  A binary material gate plus
one signed regression head can average opposing homotopies and activate a
large signed correction in ordinary states.  The next bounded experiment must
test an explicit left/anchor/right mixture offline before any runtime wiring.

No residual checkpoint was promoted and no residual closed-loop run was made.
