# Evidence

## Immutable inputs

- production checkpoint:
  `checkpoints/20260901_055824/candidate.npy`
- checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- production corpus: `dagger_aggregate_v2`, 4 sequences, 6000 sampled states
- admitted successor corpus: `precontact_residual_base_v4`, 10 sequences,
  13316 sampled states
- failures:
  - `output/20260901-152109/d1`
  - `output/20260901-153143/d1`
  - `output/20260901-153143/d3`

The generated full report is
`output/20260901-e2e-state-coverage-audit.json` and remains outside git.

## Results

| failure | production geometry above p95 | production embedding above p95 | teacher geometry above p95 | material teacher correction | local action ambiguity |
|---|---:|---:|---:|---:|---:|
| NPC d1 | 0.0% | 100.0% | 0.0% | 100.0% | inconclusive: 1.0 usable run |
| peer d1 | 0.0% | 21.6% | 0.0% | 100.0% | 51.1%, 7.82 usable runs |
| peer d3 | 100.0% | 1.1% | 37.9% | 94.3% | inconclusive: 0.85 usable runs |

NPC d1 is not a simple absence of coarse obstacle geometry: the successor
teacher corpus has close physical neighbours, but the frozen policy embedding
places every query beyond the production cross-run p95.  The successor teacher
requests a 0.639 rad mean absolute correction on every query sample.  This is a
representation/training-support gap, not evidence for a brake threshold patch.

Peer d1 is physically covered and has neighbours from nearly all teacher runs,
yet 51.1% of query states have both positive and negative material teacher
actions among close single-frame observations.  Its direct teacher correction
opposes the frozen base steering sign on 92.0% of samples.  This is strong
evidence that a single scan cannot safely select the action without temporal or
state context.

Peer d3 is a genuine coverage gap in physical LiDAR geometry.  All query states
are beyond the production cross-run p95, and 37.9% remain beyond the admitted
teacher p95.  The frozen embedding nevertheless treats almost all of them as
in-distribution, demonstrating representation collapse rather than successful
generalization.

## Decision

Do not promote candidate5/6, residual, or recurrent candidates and do not
change runtime thresholds.  The next bounded Slice is a data representation
contract:

1. preserve per-beam physical geometry instead of relying on compact pressure
   tokens alone;
2. add an allowed temporal/state discriminator for peer d1;
3. collect seed-disjoint teacher coverage for peer d3-like states;
4. require the same coverage audit to improve before any closed-loop run.

Existing recurrent experiments are not reusable admission evidence for this
step: direct GRU and compact adapters failed offline generalization.  The next
candidate must first prove that it distinguishes the audited failure states
from normal anchors; more epochs on the same ten sequences are prohibited.

## Verification

```text
state coverage focused tests     5 passed
TinyLidarNet full tests          100 passed
```
