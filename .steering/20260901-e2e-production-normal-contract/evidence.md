# Evidence

## Frozen validation run

- run: `output/20260901-151131`
- status: pass
- laps: 3 / 3
- penalties: 0
- longest low-speed interval: 0 s
- longest positive-acceleration stall: 0 s
- checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

## Historical-corpus audit

Report: `output/20260901-e2e-teacher-label-consistency.json` (generated,
not committed).

- stored teacher train/validation labels reproduced exactly by the current
  candidate3 + successor-teacher implementation;
- old zero-normal train material corrections: 1907 / 13588 (14.03%);
- old zero-normal validation material corrections: 662 / 6126 (10.81%);
- every old normal source was a gap-teacher or gap-teacher-DAgger rollout.

This rules out teacher version drift and identifies the source-policy mismatch
as the invalid normal-anchor contract.

## Frozen train run

- run: `output/20260901-170521`
- status: pass
- laps: 3 / 3 (`99.760`, `88.346`, `87.626` s)
- penalties: 0
- longest low-speed and positive-acceleration stall: 0 s
- distance: 1009.06 m
- checkpoint and control mode: exact frozen identity

## Corrected immutable dataset

Generated root (gitignored): `dataset/production_normal_anchor_v1`

| split | run | samples | maximum speed sync error |
|---|---|---:|---:|
| train | `20260901-170521` | 5920 | 17.54 ms |
| validation | `20260901-151131` | 5927 | 18.38 ms |

The current successor teacher still requests a material correction for 6.13%
of train and 6.33% of validation production-normal states.  Zero residual is
nevertheless the valid preservation target because both source runs completed
without a penalty or stall under the frozen base controller.

## Offline candidate decision

The first corrected-contract candidate improved material MAE by 34.23% and
passed direction and production-normal gates, but failed teacher-neutral MAE at
0.01104 rad.  A single causal A/B changed only neutral leakage loss from 0.5 to
1.0.

Accepted for shadow evaluation only:

- artifact:
  `checkpoints/spatial-production-normal-v2/20260901_171913/candidate.npy`
- SHA-256:
  `6ae9d618ea8093b1ff7d212cae760e90c71f84749f986af479681f5f729155d1`
- material MAE improvement: 30.10%
- material sign accuracy: 84.94%
- teacher-neutral MAE: 0.00847 rad
- independent production-normal MAE: 0.00586 rad
- peer sign accuracy: 100%
- finite/bounded and embedded candidate3 identity: pass

Production remains candidate3 with residual disabled.  This slice does not
grant steering authority to the spatial candidate.
