# Tasklist

- [x] 最新走行のFollowPrepare遷移とMPCC shadowを照合
- [x] rolling replan contextをMPCC-lite prefix admissionへ接続
- [x] invalidated hold候補をfresh左右候補から除外
- [x] DynamicMissionWait中のBehavior Overtake ownershipを保持
- [x] DynamicMissionWait中の物理hold lineを出力
- [x] 単体テストを追加・更新
- [x] 対象packageをbuild/test
- [x] 次回試走の確認項目を記録

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 test targets）
- `colcon test-result --verbose`: 1106 tests、0 errors、0 failures、0 skipped
- 既存build tree内の`joycon_contract_guard/package.xml`欠損警告はあるが、対象packageの結果には影響なし

## 次回試走で確認する項目

- soft failure後に`V2X behavior: Overtake -> Follow`へ即遷移せず、rolling replan ownershipを維持すること
- `FollowPrepare + DynamicMissionWait`中もMPCC-liteのLeft/Right候補が評価されること
- fresh prefix成立時に`FollowPrepare -> ShiftOut/Pass`へ短時間で接続すること
- fresh prefix待ち中に前車速度へ急落せず、現在速度を維持すること
- hard fault、壁margin違反、EmergencyBrakeでは従来どおりRecoveryへ移ること
- FollowPrepare滞在時間、Pass完遂数、SafetyBrake回数、壁/横加速度Recovery回数を前走と比較すること
