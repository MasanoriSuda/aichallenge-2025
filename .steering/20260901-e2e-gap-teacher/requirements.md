# E2E LiDAR gap-teacher requirements

## Objective

Create an auditable LiDAR-only avoidance teacher on top of the admitted single-vehicle TinyLidarNet
lane-following policy. Use it only to collect corrective steering labels for runtime NPC encounters.

## Constraints

- Keep `control_method=tiny_lidar_net`; do not add a submission interface value.
- `gap_teacher` is an explicit launch/runtime mode and is never the production default.
- Teacher inputs are only the current 180-degree 2D LiDAR scan and the base network output.
- No GNSS, IMU, V2X, map pose or trajectory may enter the teacher policy.
- A run is extracted only after Finish/contact/stall admission.
- The final student still publishes ML steering. A rule-based teacher is not a submission candidate.
- Existing production checkpoint is not overwritten until closed-loop gates pass.

## Definition of Done

1. Gap selection and command blending are deterministic and unit-tested.
2. Launch overrides are explicit, validated and default to the existing `fixed` mode.
3. Runtime NPC teacher completes without positive-acceleration stall.
4. At least two admitted seeds exist before train/validation extraction.
5. Label provenance is `lidar_gap_teacher` and cannot be mistaken for MPC or student data.
6. A trained candidate passes offline validation, `e2e-single`, and runtime NPC A/B before promotion.
