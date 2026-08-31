# E2E longitudinal safety requirements

## Objective

Close the deployed-student contract exposed by NPC seed 2027: TinyLidarNet owns
lateral control, while an auditable LiDAR safety layer may reduce only the fixed
longitudinal acceleration before physical embedding.

## Constraints

- Do not change the production checkpoint.
- Do not alter TinyLidarNet steering in the production safety mode.
- Reuse the teacher's exact frontal percentile and stop/slow thresholds.
- Keep `gap_teacher` as an explicit teacher-only lateral mode.
- Fail closed for stale LiDAR through the existing watchdog.
- Preserve `/control/command/control_cmd` and launch/topic contracts.
- Log longitudinal safety activation separately from teacher activation.

## Definition of Done

1. Teacher and production safety call one longitudinal policy implementation.
2. Clear LiDAR preserves fixed acceleration and network steering bit-for-bit.
3. Slow/stop observations produce zero/braking acceleration without changing steer.
4. Unit, launch, system-launch and build checks pass.
5. Single vehicle and NPC seed 2027 gates pass before any production weight change.
