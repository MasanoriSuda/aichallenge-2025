# Requirements

## Objective

Track/Cruise MPCC shadow の physical wall certificate reject を、次の発生源へ分離する。

- legacy production control が作った現在姿勢の wall unsafe
- MPCC candidate の離散 horizon pose の wall unsafe
- 現在姿勢から MPCC horizon までの swept segment の wall unsafe

## Root cause being addressed

`SweptFromCurrentPose` の path index 0 は `actual_wall_monitor_pose_` だが、現行診断は
index 0 reject 時に最後に検査した horizon stage の値を保持する。そのため現在姿勢の接触が
`stage=19` の MPCC candidate failure に見え、Slice 2 の阻害要因を誤分類している。

## Constraints

- shadow authority (`authority=shadow, selected=0`) を変更しない。
- physical certificate の合否を緩和しない。
- wall footprint、margin、solver formulation、command を変更しない。
- `aichallenge/result-summary.json` を変更・stageしない。

## Definition of Done

- path index 0 の current pose failure が専用reasonで記録される。
- horizon stage / swept segment failure が current pose と混同されない。
- aggregate telemetry で current pose reject を独立集計する。
- failure-first unit test、package build/test、shadow試走が通る。
- Slice 2 の昇格判定が candidate defect と predecessor-state defect を区別する。
