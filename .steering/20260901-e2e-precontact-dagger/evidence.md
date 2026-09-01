# Evidence

## Source admission

- source run: `output/20260901-100204`, d1--d4
- source checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- teacher: `LidarPrecontactTeacher`
- control mode: `precontact_teacher`
- label source: `lidar_precontact_teacher_dagger`

Every sequence records the source bag, teacher configuration, control mode,
checkpoint hash and pre-contact cutoff.  All four bags had no confirmed range
breach below 0.5 m and retained their complete recorded prefix.

## All-active extraction (rejected training design)

The first extraction kept every active pre-contact-teacher decision:

| Domain | Accepted labels |
|---|---:|
| d1 | 1,403 |
| d2 | 1,708 |
| d3 | 1,376 |
| d4 | 930 |
| Total | 5,417 |

Only 561 samples differed at all from the historical `LidarGapTeacher`; 418
differed by at least 0.02 rad.  The remaining samples repeated an already learned
gap policy.  Candidate4 (`checkpoints/20260901_102150`) trained on all 5,417
labels and was rejected before closed-loop testing:

- independent validation MAE: 0.015784 -> 0.020228 rad (28.2% worse)
- independent validation RMSE: 0.041860 -> 0.043742 rad (4.5% worse)

## Successor-only extraction

`--novel-policy-only` compares the two teachers on the same scan and base
steering.  It retained only material successor-policy deltas:

| Domain | Delta >= 0.02 rad |
|---|---:|
| d1 | 121 |
| d2 | 73 |
| d3 | 135 |
| d4 | 89 |
| Total | 418 |

The candidate5 aggregate contains 14,010 train samples in seven run-identified
sequences.  The original independent validation remains one sequence with 6,130
samples; sequence leakage is rejected by the loader.

## Candidate comparisons

Candidate5 (`checkpoints/20260901_102515`, full-network fine tuning):

- SHA-256: `f84e802fc6976906ddb062e1f5ddd509119a39602b55051b25e519b761c0f9e3`
- new correction MAE: 0.245721 -> 0.115230 rad (53.1% better)
- independent validation MAE: 0.015784 -> 0.019189 rad (21.6% worse)
- independent validation RMSE: 0.041860 -> 0.044219 rad (5.6% worse)

Candidate6 (`checkpoints/20260901_102724`, only `fc4` trainable):

- SHA-256: `5c152eb021440aa054b5fba14caf43d830dfe6a0b52cf3d08b22d18ba358f74c`
- new correction MAE: 0.245721 -> 0.239254 rad (2.6% better)
- independent validation MAE: 0.015784 -> 0.017578 rad (11.4% worse)
- independent validation RMSE: 0.041860 -> 0.041972 rad (0.27% worse)

Candidate6 preserves the base representation but cannot express the desired
correction.  Candidate5 demonstrates the correction and advances only as a
diagnostic closed-loop candidate; neither replaces production at this point.

## Candidate5 single-vehicle gate

- run: `output/20260901-102829`
- terminal: AWSIM `Finish`
- distance: 1,012.25 m
- duration: 300.12 s
- mean forward speed: 3.384 m/s
- longest post-start low-speed interval: 0.0 s
- longest positive-acceleration stall: 0.0 s
- scan stale count: 0

The normal closed loop remains viable despite the offline regression.  The next
decision gate is the four-peer final world, where the new labels are relevant.
Production authority and the shipped checkpoint remain unchanged.
