# Evidence

## Source admission

The strict competition source is the validation-only run
`output/20260902-e2e-final-speed-committed-validation-v1`, whose report SHA-256
is `3b575aa9a0f5b75629f379ed3cd1cd6167d3fa2d4a9c76c7dc116e2eb166697f`.

Domains d1, d2 and d4 passed the 0.100 s causal speed freshness contract and
produced 13,325 validation samples in total:

| Domain | Samples | Material | Maximum speed age [s] | Raw metadata SHA-256 |
|---|---:|---:|---:|---|
| d1 | 4,519 | 1,547 | 0.057690 | `63f8eca7ec2649c005eb8dde40c1e44b1196ab16c54e34714642ea17233d9d55` |
| d2 | 4,417 | 1,447 | 0.057288 | `c0f545a775e0be2205423fdc080319eff1790bffeffb6e9cb892f4e9ecf5c072` |
| d4 | 4,389 | 857 | 0.056544 | `cb33f05be301f796fe23980093499b0094c9e971f88317423b3a2dcca706306d` |

d3 was rejected as a dataset source.  One scan at index 1,652 had a latest
preceding wheel-speed age of 102.271469 ms.  The following speed message was
recorded 31.289 microseconds after the scan.  This may be cross-topic recorder
ordering rather than runtime callback order, but bag timestamps cannot prove
that.  Skipping the middle scan would corrupt stateful teacher continuity and
raising the 100 ms limit would change the executed contract, so the complete d3
sequence remains excluded.

## Validation-only derivative

The recurrent builder previously required train and val together.  An explicit
`--split val` contract was added while keeping the historical default unchanged.
The resulting manifest records zero train sequences, three val sequences and
13,325 samples.  Manifest SHA-256:
`1217a469e987b3be410522a901025697a8f455249f2c4886ef4c1427d8445b29`.

## Candidate comparison

Every model was decoded with a 0.02 rad correction deadband and evaluated with
the same frozen base, frozen production spatial model, 0.100 s freshness limit
and independent production-normal corpus.  Lower MAE is better.

| Model | Overall MAE [rad] | Material MAE [rad] | Material improvement | Anchor MAE [rad] | Normal correction MAE [rad] | Existing gate |
|---|---:|---:|---:|---:|---:|---|
| frozen production recurrent | 0.033551 | 0.092572 | 8.22% | 0.015279 | 0.001966 | fail |
| peer-64 | 0.026238 | 0.053005 | 47.45% | 0.017952 | 0.002920 | fail |
| peer-512 | **0.018237** | **0.047033** | **53.37%** | **0.009322** | **0.001076** | pass |
| run-balanced peer-64 | 0.020234 | 0.050644 | 49.79% | 0.010820 | 0.001099 | pass |

Peer-512 was best on aggregate and on each admitted peer domain.  Its report
SHA-256 is
`a770fb2075c4b81320910f45de349594e108919d0efa849f44328689daeb8552`.
Production, peer-64 and run-balanced report SHA-256 values are respectively
`293e6171e4b4a08ffcfa2d225c5b2f6f6d1019b9f18110658ac7ce433a2f8ce0`,
`1550900ea4f0a803a4cc53b749aff81d7e3862d4ba5ab0350b5dbe0f2db1fcb6`
and `0977de997af9a46615ca90ba603aa87e8b9e0f2fc0ad250e114cfb2b8e0dd5d4`.

## Decision

Promote peer-512 only to the next *runtime shadow* review.  It is not granted
steering authority: the new validation is execution-disjoint but uses the same
deterministic world as its peer training source, and historical seed2033
material MAE was still 0.73% worse than the old candidate.  A closed-loop run
must therefore keep production commands unchanged while checking load identity,
inference coverage, timing, finite output and hidden-state reset behavior.

Training datasets, packaged checkpoints, launch defaults and ROS output
authority were unchanged in this Slice.
