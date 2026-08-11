# Tasklist

- [x] 直前ログの微小ShiftOut失敗を特定する
- [x] 現行DirectPassとMission距離計算を確認する
- [x] pure core DirectPass資格policyを追加する
- [x] candidate生成とentry stageへ接続する
- [x] dev/cloud configと起動ログを追加する
- [x] pure core testを追加する
- [x] 対象package testを実行する
- [x] `make autoware-build`を実行する
- [x] 差分と外部interface非変更を確認する

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25成功
- `colcon test-result --verbose`: 1023 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
- dev/cloud config: `0.20 m`で一致
- topic / service / message / launch契約の変更なし

## Dynamic acceptance（ユーザー試走）

- 起動ログに`tiny-shift direct Pass<=0.20 m`が出ること
- 対象Missionが`Idle -> Pass`、reason=
  `validated tiny-shift corridor already clear`になること
- 対象地点でSafetyBrake、Recovery、Reverseへの連鎖が消えること
- 0.20 m超のMissionが引き続きShiftOutとなること
