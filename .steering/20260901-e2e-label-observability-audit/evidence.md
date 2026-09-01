# Evidence

## Frozen inputs

- Production candidate: v11 spatial steering authority
- Candidate SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- Teacher corpus: current successor-teacher train and validation sequences
- Normal corpus: three admitted production-normal sequences
- Material correction threshold: `abs(delta) >= 0.02 rad`
- Audit output: `output/20260901-e2e-spatial-label-observability.json`

The audit sampled 4,096 material teacher states from 4,358 available material
states and 2,048 normal queries.  Natural distance was derived only from
nearest neighbours in a different successful normal run, so adjacent frames
from the same run cannot make the reference envelope artificially small.

## Cross-label overlap

| Representation | Normal cross-run p50 / p95 | Teacher material inside p50 / p95 | Normal inside material p50 / p95 |
|---|---:|---:|---:|
| exact v11 adapter input | 2.7278 / 7.9867 | 8.23% / 29.32% | 7.62% / 44.04% |
| physical binned geometry | 0.09163 / 0.43434 | 3.20% / 19.80% | 2.98% / 28.08% |

The frozen random projection roughly doubles the strongest conflict region,
so representation compression contributes to the normal leakage.  It is not
the sole root cause: even physical LiDAR geometry, speed and base steering put
material teacher labels inside the natural variation of successful zero-label
normal runs.

On the audited material states, v11 retained 99.15% sign accuracy with target
mean magnitude 0.23590 rad, prediction magnitude 0.21665 rad and MAE 0.05828
rad.  The nearest normal states nevertheless had a mean predicted correction
of 0.04225 rad, while sampled normal queries averaged 0.01050 rad.  This is
consistent with the 5--9% normal false-material rate seen in prior candidate
audits.

## Geometry control probe

The physical binned geometry was also evaluated as a classifier input over
three independent seeds.  It did not qualify as a replacement representation.

| Seed | Variant | Balanced accuracy | Material sign | Normal false material | Focus sign | Focus-tail sign |
|---:|---|---:|---:|---:|---:|---:|
| 2026 | exact spatial + base | 0.89362 | 0.89904 | 0.09010 | 0.91007 | 1.00 |
| 2026 | geometry + base | 0.88973 | 0.90775 | 0.10697 | 0.89029 | 1.00 |
| 2027 | exact spatial + base | 0.90384 | 0.91123 | 0.09684 | 0.92266 | 1.00 |
| 2027 | geometry + base | 0.88110 | 0.89469 | 0.09229 | 0.89209 | 0.00 |
| 2028 | exact spatial + base | 0.89974 | 0.90339 | 0.08183 | 0.89388 | 1.00 |
| 2028 | geometry + base | 0.89598 | 0.91384 | 0.10612 | 0.89928 | 0.50 |

Geometry loses balanced accuracy in all three seeds, increases normal false
material actions in two seeds, and is unstable on the frozen failure tail.  A
new production candidate was therefore not trained.

## Root-cause classification

The failure is mixed but now bounded:

1. The v11 compressed representation collapses some otherwise distinguishable
   geometry and makes label conflict worse.
2. A substantial conflict remains before that compression.  The same static
   physical observation contract is assigned a material teacher correction in
   one corpus and zero intervention in admitted successful normal data.
3. A short causal history and an explicit categorical neutral head were already
   rejected by independent slices, so neither is justified as the next patch.

The next root-cause action is a teacher/normal label-contract audit.  It must
identify conflict at immutable sequence/sample level and determine whether
ambiguous teacher samples can be excluded without removing the frozen
collision-avoidance tail.  Runtime triggers, correction thresholds, model
authority and the production v11 artifact remain frozen.
