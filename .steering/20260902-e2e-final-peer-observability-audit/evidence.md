# Evidence

## Frozen inputs

- report:
  `output/20260902-e2e-final-peer-observability-v2.json`
- report SHA-256:
  `84baa346c47956c806f153cb4079c587c9d60564822b9d1e2d183c7950c5bd1c`
- production spatial-v11 SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- recurrent corpus manifest SHA-256:
  `6f769c1d10d4662d371e61ea2aa6ffa32e970e7a6233923931b79f295d336190`
- speed freshness contract: explicitly 0.100 s
- deterministic audit sample: 9,000 teacher states, 2,083 material states,
  and 4,500 successful normal states

## Aggregate overlap

Material states are compared with their nearest successful-normal state.  The
threshold is the natural cross-run distance between independent normal runs.

| Representation | Inside normal p50 | Inside normal p95 |
|---|---:|---:|
| projected v11 input | 8.21% | 46.14% |
| full conv5 | 5.86% | 39.65% |
| physical binned geometry | 3.55% | 26.79% |

The previous precontact-teacher audit measured 8.23%/29.32% for projected v11
and 3.20%/19.80% for physical geometry.  Strong overlap remains similar while
the broader p95 region grows.  Compression contributes but is not the only
factor.

## Distribution split hidden by the aggregate

Physical-geometry p50 overlap by source:

| Source | Material queries | Inside normal p50 |
|---|---:|---:|
| final peer d1 | 466 | 0.00% |
| final peer d2 | 499 | 0.00% |
| final peer d3 | 351 | 0.00% |
| final peer d4 | 315 | 0.63% |
| historical seed2034 | 247 | 13.77% |
| historical seed2033 validation | 205 | 18.54% |

The same split appears in projected-v11 space: final peer domains range from
0.60% to 7.94%, while historical seed2034/2033 are 20.65% and 26.83%.

Thus the peer labels themselves are not the main ambiguous subset.  They are
more observable from LiDAR geometry than the old single-world labels.  The
offline candidate comparison nevertheless selects on seed2033 and audits on
seed2035, both from the historical distribution.  All four new peer domains
belong to one correlated training run, so there is no independent peer-world
generalization measurement.

## Stateful replay correction

The first report exposed a diagnostic defect: the audit replayed the current
stateless precontact teacher over labels produced by the stateful
speed-committed teacher.  The resulting 0.83 rad mismatch was not model drift;
the replay contract was invalid.

The audit now records the unique recurrent label source and marks teacher and
counterfactual-normal replay `not_applicable` for stateful speed-committed
labels.  Deterministic subsampling cannot reconstruct its hidden commitment
state.  Precontact-teacher datasets retain the existing replay path.  This
prevents future reports from treating two different teachers as an identity
failure.

## Root-cause classification

The controlled experiments and observability audit support this chain:

1. adding one correlated peer world broadens the training distribution;
2. the 64-unit model lacks capacity and regresses broadly;
3. 512 units recover aggregate and normal behavior;
4. neither natural nor source-run balanced training improves material MAE on
   both historical validation worlds;
5. those worlds contain substantially more observation/label ambiguity than
   the new peer world;
6. no independent peer world exists to measure the intended improvement.

The next action is data/evaluation, not another loss or sampling tweak.  Run
the already-qualified speed-committed teacher in a new independent four-domain
world, certify it, and reserve it as validation-only.  Until then, no peer
candidate should be converted or connected to runtime authority.

## Verification

- focused audit/recurrent tests: 37 passed
- complete TinyLidarNet workspace suite: 226 passed
- Python byte-code compilation: passed
- `git diff --check`: passed
- runtime topics, launch, packaged checkpoints and authority: unchanged
