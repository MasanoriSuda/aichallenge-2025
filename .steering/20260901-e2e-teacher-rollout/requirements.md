# E2E teacher rollout requirements

## Objective

Transfer the closed-loop obstacle behavior demonstrated by the admitted gap
teacher into a TinyLidarNet student without changing runtime lateral authority.

## Evidence entering this slice

- Candidate2 stalls on NPC seed 2027 after 878.89 m.
- The longitudinal safety layer prevents positive acceleration but cannot create
  a lateral escape.
- The gap teacher passes the exact world for 1020.22 m with no post-start stall.

## Constraints

- Never use failed student commands as labels.
- Admit only the pre-stop teacher rosbag with explicit
  `lidar_gap_teacher` provenance.
- Preserve the existing independent validation run.
- Do not change the runtime controller or production checkpoint in this slice.
- Reject a candidate that fails either the reproduced seed or an unseen seed.

## Definition of Done

1. The complete successful teacher rollout is extracted as a train-only sequence.
2. Candidate3 improves the corrective subset without material normal-run regression.
3. Candidate3 passes single vehicle, seed 2027 and one unseen NPC seed.
4. Every dataset and checkpoint is traceable in the evidence document.
