# Evidence

Generated run (not committed):
`output/20260902-e2e-certified-precontact-seed2032`.

## Immutable setup

- world: one ego and two NPCs (`e2e-npc-single`);
- random seed: 2032;
- runtime mode: `precontact_teacher` with no provenance conflicts;
- base checkpoint:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.

## Result

| Metric | Value |
|---|---:|
| laps | 104.302 / 90.070 / 110.709 s |
| total lap time | 305.081 s |
| Finish | 3/3 laps |
| penalty | 1 crash, 10.125 s |
| distance | 1,022.567 m |
| mean / max speed | 3.1569 / 4.4281 m/s |
| low-speed / positive-accel stall | 0 / 0 s |
| minimum front LiDAR range | 0.0350 m |

The motion gate passed, but the strict competition gate failed with
`penalty-limit-exceeded`. Artifact hashes are:

- competition analysis:
  `80a1949beb50fb7a46f138cceb9a60cf9f0228c16d2d924a4f47aef77051835e`;
- result summary:
  `91dfd995b24fc502b394c8fb98c52c97421c7a8c08af8e806a892df8dabcd6b8`;
- result detail:
  `2718039047fecc29730d270978c55acbab6fe1931a1cf9f204ca68ae55242121`;
- motion analysis:
  `694904a707a758d618b898b9c66cd2cc16fe591005be4073321bb6bc6187b3c6`.

## Contact reconstruction

At approximately 274.50 s after the first scan, the vehicle still travelled at
3.786 m/s with 1.998 m frontal clearance and only 1.074 m on the left. Within
the next 0.40 s the teacher switched from near-neutral steering to the opposite
side and saturated at +0.64 rad while commanding -1.0 m/s2. Frontal clearance
fell through 1.531, 1.147, 0.827, 0.613, 0.397 and 0.217 m before reaching
0.035 m. Speed was still 1.281 m/s at minimum clearance.

Successful seed 2031 also encountered sub-metre side clearance, but its nearest
third-lap approach retained 5.4--7.4 m frontal clearance and passed without the
late cross-side reversal. The failing seed therefore exposes a policy defect:
the heuristic can change gap side after the remaining longitudinal distance is
already insufficient for either lateral escape or braking. It is not a model
capacity result and is not repaired by relabeling this failed rollout.

## Decision

Seed 2032 is held-out failure evidence and is not extracted as a hard teacher
validation sequence. One successful seed is insufficient for run-disjoint
training. The next slice must replace the late reactive teacher decision with
a speed-aware, temporally committed teacher candidate and certify that policy
before generating more hard labels.
