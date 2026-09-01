# Evidence

## Independent train run

`output/20260902-e2e-speed-committed-seed2034` executed the frozen
`speed_committed_teacher` with random seed 2034.

| Metric | Value |
|---|---:|
| laps | 104.777 / 89.620 / 95.847 s |
| total lap time | 290.244 s |
| Finish | 3/3 |
| penalty | 0 |
| distance | 1,024.582 m |
| mean / max speed | 3.3180 / 4.4418 m/s |
| low-speed / positive-accel stall | 0 / 0 s |
| minimum front LiDAR | 2.1087 m |

Artifact SHA-256 values:

- competition analysis:
  `70b1f6714b6d15f430c20f034578c91a19e4f12486dc622296f20f162c2cce8f`;
- result summary:
  `8fbf03c7a154879e954fc62094d6dbbb9c089c4305968cff8af97bfe684b2154`;
- result detail:
  `29a34d10f0d2f248b4d2d5457cbc492ac63cc65fb91ef6e9963d5db03498feee`;
- motion analysis:
  `83bd3b0758820614fe7b27a3441ed84a0201d1862d4e471d060a26ee0a078fad`.

The embedded executed-teacher certificate SHA-256 is
`3d952cd668e44ec20d69c6dd8a106f004dcb7783dff7a3ad81f11b71da7a851c`.

## Run-disjoint data

- train seed 2034: 6180/6180 scans, max causal speed age 0.038089130 s;
- validation seed 2033: 6100/6100 scans, max causal speed age 0.040296361 s;
- train/validation sequence identity overlap: none;
- recurrent label source:
  `lidar_speed_committed_teacher_recurrent_direct`;
- aggregate manifest SHA-256:
  `3fbf4d10310527841d99448c5a79332bbb885a8bc4fb6f453109a8261d45d6c0`.

The builder accepts separately rooted immutable train and validation evidence,
but rejects an aggregate missing either split and rejects duplicate identities.

## Held-out representation probe

Probe report:
`output/20260902-e2e-speed-committed-representation-probe.json`
(SHA-256
`08e3750be68969c86afccb2f194c68dafe5d8901f462c516c935fc690686ce96`).

Training used seed 2034 only. All reported metrics below are seed 2033.

| Representation | accuracy | balanced | material sign | neutral false material |
|---|---:|---:|---:|---:|
| static compact fc3 + speed | 0.6846 | 0.6138 | 0.5691 | 0.2968 |
| static projected conv5 + speed | 0.8936 | 0.8944 | 0.8949 | 0.1066 |
| temporal projected conv5 + speed | 0.9162 | 0.8960 | 0.8831 | 0.0784 |
| temporal projected conv5, no speed | 0.9187 | 0.8991 | 0.8867 | 0.0761 |
| static raw LiDAR + speed | 0.8523 | 0.8401 | 0.8323 | 0.1445 |
| temporal raw LiDAR + speed | 0.9033 | 0.8620 | 0.8359 | 0.0859 |

Temporal projected spatial features reduce neutral false corrections by 2.95
percentage points and improve overall accuracy by 2.51 points over the static
projected feature. Adding scalar speed to the temporal probe does not improve
the held-out result. The major separator is spatial conv5 information plus
history; the 10-dimensional compact fc3 representation loses too much
geometry.

## Decision

Use a frozen-base recurrent adapter whose trainable path receives projected
conv5 spatial features and causal history. Do not select the compact-fc3 GRU,
raw direct-steering GRU or an extra scalar-speed-only patch from this evidence.

This probe is not a promotion gate: it does not include an independent normal
anchor corpus and predicts correction class rather than steering magnitude.
The selected adapter must still pass regression, normal leakage, finite-output
and closed-loop gates. Production remains frozen.

## Verification

- focused source-aggregation tests: 21 passed;
- full ML test suite: 194 passed;
- strict seed-2034 competition gate: pass;
- raw-to-recurrent train/val derivation: pass;
- representation probe: completed on CUDA.
