# E2E final contact teacher A/B requirements

## Objective

Determine whether the four-vehicle final failure is correctable by the admitted
LiDAR gap teacher before collecting or training any additional student data.

## Constraints

- Keep the world, starts, peers, checkpoint and longitudinal policy identical to
  `output/20260901-085903`.
- Change only domain 4 lateral authority to teacher-only `gap_teacher`.
- Domains 1--3 remain production `fixed_lidar_brake` students.
- Do not change production launch defaults, weights or safety thresholds.
- Do not extract labels unless the teacher domain passes the run-level stall and
  contact/Finish evidence gates.

## Definition of Done

1. A reproducible Make target identifies the teacher domain explicitly.
2. All four runtime modes are visible in their launch logs.
3. Domain 4 is compared against production d4 using the same analyzer.
4. The result decides between a corrective-data slice and a teacher redesign.
