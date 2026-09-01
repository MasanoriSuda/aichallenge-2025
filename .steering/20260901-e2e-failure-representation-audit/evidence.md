# Evidence

## Immutable sources

- train-only successful authority run:
  `/output/20260901-175609/d1/rosbag2_autoware`
- validation-only failed NPC authority run:
  `/output/20260901-180313/d1/rosbag2_autoware`
- frozen candidate3 SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- admitted failure prefix ends one second before the first confirmed 0.5 m
  LiDAR breach; the failed run never enters training.

The relabelled sources are in the ignored dataset roots
`spatial_authority_teacher_v1` and `recurrent_direct_v4`.  The persisted report
is `output/20260901-e2e-failure-representation-probe.json`.

## Representation result

| Representation | Aggregate balanced accuracy | Aggregate material sign | Failure-prefix material sign | Last-200 material sign |
|---|---:|---:|---:|---:|
| frozen conv5 + speed | 0.905 | 0.895 | 0.919 | 0.924 |
| frozen conv5, no speed | 0.898 | 0.890 | 0.902 | 0.924 |
| frozen compact fc3 + speed | 0.637 | 0.599 | 0.623 | 0.883 |
| raw scan + speed | 0.857 | 0.833 | 0.894 | 0.878 |
| frozen conv5 short history | 0.883 | 0.863 | 0.874 | 0.924 |
| raw scan short history | 0.849 | 0.821 | 0.834 | 0.843 |

Raw LiDAR through a generic MLP is not an improvement over the frozen spatial
feature.  Adding causal lags 1 and 8 also does not improve direction
classification.  The compact candidate3 bottleneck remains clearly
insufficient, but the frozen conv5 representation can distinguish the required
correction direction in this held-out failure.

## Failure-tail audit

The last 200 samples cover 9.96 seconds at 19.96 Hz.  Their target support is:

- left correction: 98;
- neutral: 3;
- right correction: 99.

This is not frame-to-frame label chatter.  There are two long material runs:
98 samples in one direction, three neutral samples, then 99 samples in the
other direction.  Replaying `LidarPrecontactTeacher` shows that the change is
driven by a coherent geometry transition: the first correction reduces the
base steering while the front range is about 4.4 m, then the selected opening
and side-clearance geometry require the opposite correction as the right-side
return approaches.

`anchor_false_material_fraction=1.0` in the tail means the three neutral
targets were all classified as material; it does **not** mean material targets
were missed.  Material direction accuracy is the relevant tail metric.

## Decision

Keep production and the bounded spatial-authority experiment frozen.  The
evidence rejects both of these immediate changes:

- replacing conv5 with a raw-scan MLP;
- adding only short finite differences to the frozen representation.

The next controlled experiment is a DAgger-style retraining comparison using
the successful authority run as train-only and the failed authority prefix as
held-out validation.  It must establish whether the existing spatial head is
merely missing closed-loop coverage.  A new temporal architecture is justified
only if retraining the same architecture still cannot improve the held-out
failure without degrading production-normal anchors.  No model from this audit
is a runtime checkpoint.

## Verification

- targeted probe unit tests: 9 passed in the development container;
- deterministic probe completed for all seven representations;
- train/validation sequence identities are disjoint;
- raw scans remain physical metres at dataset boundaries and are normalized
  exactly once inside each probe.
