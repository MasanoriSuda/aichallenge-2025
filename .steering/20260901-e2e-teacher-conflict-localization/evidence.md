# Evidence

## Frozen audit

- production v11 SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- teacher: `recurrent_direct_v6_wheel_speed_seed2030`
- normal: `production_normal_anchor_v3_wheel_speed_current`
- output: `output/20260901-e2e-spatial-label-conflict-localization.json`
- all 4,358 sampled material teacher labels were retained;
- final 200 source samples were reported independently per sequence.

The current `LidarPrecontactTeacher` reproduced stored teacher corrections with
maximum absolute error `4.77e-7 rad`.  The conflict is therefore not teacher
version drift.  Replaying the same teacher on sampled admitted normal states
requested a material correction on 279/4,500 states (6.20%): 169
`side-clearance` and 110 `gap-selected` decisions.  Stored normal labels are
exactly zero.

## Localized conflicts

| Representation | Material inside normal p50 | Material inside normal p95 |
|---|---:|---:|
| exact v11 adapter input | 8.26% | 29.23% |
| physical binned geometry | 3.24% | 19.83% |

The 360 strongest exact-input conflicts consist of 201 `side-clearance` and
159 `gap-selected` labels.  Both teacher mechanisms contribute; this is not a
single erroneous decision branch.

The immutable closest pairs include corrections as large as `-0.385 rad` and
`+0.229 rad` whose exact v11 input lies almost on top of an admitted normal
sample.  The JSON records teacher/normal sequence ID, source bag and original
sequence-local sample index for reproduction.

## Why conflict filtering is rejected

The four-vehicle failure run `/output/20260901-121938/d2` is decisive.  In its
final 200 material samples:

- exact projected v11 input: 60% inside normal p50, 100% inside p95;
- physical binned geometry: 0% inside normal p50, 100% inside p95.

An exact-input p50 filter would delete 120/200 corrections from a known
failure tail that needs lateral authority.  This would optimize clean-track
leakage by erasing safety-critical supervision.  A physical-p50 filter retains
that tail, but the current production representation has already discarded the
distinction and therefore cannot exploit the filter by itself.

The frozen unseen focus run `/output/20260901-130837/d1` has one material
sample in its final 200 source samples.  It is outside p50 but inside p95 in
both representations.  Broad p95 filtering is therefore also unsafe for the
admission Gate.

## Decision

Do not filter/relabel data and do not train another projected-v11 candidate.
The correct next independent variable is the 1,088-to-128 random projection in
the spatial adapter.  Evaluate the full frozen conv5 map, with the same speed,
base steering, split, classifier and frozen Gates, against projected v11.

Only if the full conv5 representation reduces cross-label overlap and improves
three-seed action separation without weakening four-peer/focus tails may a new
offline steering adapter be trained.  Production runtime, v11 authority,
teacher thresholds and normal labels remain unchanged.
