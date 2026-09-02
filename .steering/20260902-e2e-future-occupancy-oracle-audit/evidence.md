# Evidence

## Frozen comparison

Artifact:

`output/20260902-e2e-peer-speed-committed-teacher/future-occupancy-maneuver-audit.json`

The exact same rollout bank was evaluated with future LiDAR scans transformed
through recorded future ego poses.

| case | any candidate | no candidate | selected invalid / opposite valid |
|---|---:|---:|---:|
| successful train | 13.89% | 86.11% | 5.56% |
| successful validation | 17.48% | 82.52% | 9.71% |
| failed mixed-peer last 20 s | 94.12% | 5.88% | 17.65% |

The failed interval has a larger wrong-side signal, but the frozen candidate
bank cannot represent a majority of either successful run.  Its Stop suffix
uses zero steering after the lateral manoeuvre, so at racing speed it often
runs straight into future curved-track occupancy.  Recorded future scans also
come from the actual ego path, not a counterfactual candidate sensor view.

## Decision

Classification: `inconclusive-candidate-bank-misses-success`.

- Temporal occupancy contains an exploratory signal, not an admissible label.
- The result does not justify recurrent training or runtime authority changes.
- Clearance thresholds must not be relaxed to make this audit pass.
- A future teacher needs a course-following contingency/successor and valid
  counterfactual geometry, or genuinely successful privileged demonstrations.
- Production remains unchanged.
