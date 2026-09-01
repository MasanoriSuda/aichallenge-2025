# Evidence

## Frozen production identities

- raw TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- packaged v11 spatial adapter SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- projected-conv5 recurrent candidate SHA-256:
  `bcd1652a31215be58b258b66fb301884863d3d2c1179932b35b1d05079a21304`

The evaluator reloaded both production artifacts independently and compared
every embedded tensor.  Both identity gates passed.

## Data boundary

- train: speed-committed teacher seed 2034, 6,180 ordered samples
- development validation: seed 2033, 6,100 ordered samples
- final unseen audit: seed 2035, 6,174 ordered samples
- independent normal validation: frozen production run, 5,927 ordered samples

Every teacher run carries an `executed_teacher_success` certificate.  Seed
2035 completed 3/3 laps with zero crash, wall and course-out penalties, and
zero post-start stall duration.  It was collected after the 0.02 rad deadband
was fixed and was not used for training or threshold selection.

## Final unseen gate

Report:
`output/20260902-e2e-conv5-recurrent-seed2035-gate.json`

| Metric | Frozen production | Candidate | Result |
|---|---:|---:|---|
| all-sample MAE | 0.017160 rad | 0.013821 rad | 19.46% improvement |
| material MAE | 0.094450 rad | 0.054414 rad | 42.39% improvement |
| anchor MAE | 0.001158 rad | 0.005416 rad | below 0.01 rad gate |
| independent-normal correction MAE | - | 0.001966 rad | below 0.01 rad gate |

All finite/bounded, unseen-not-worse, material-improvement, normal-leakage and
embedded-identity gates passed.

## Tests

- focused recurrent suite: 27 passed
- complete TinyLidarNet workspace suite: 200 passed
- Python byte-code compilation: passed
- pre-change compact recurrent checkpoint: strict-load passed
- `git diff --check`: passed

The host and development image do not currently contain the `pre-commit` or
`black` executables, so those optional wrappers could not be run.  The
repository hook set does not define a Python formatter; the executable test
and compile checks above cover the changed Python modules.

## Decision

The adapter is admitted as an **offline shadow candidate only**.  Production
authority remains the packaged v11 spatial policy.  Runtime integration needs
a separate slice covering hidden-state lifecycle, speed freshness, watchdog,
raw-correction deadband decode and closed-loop single/NPC acceptance.
