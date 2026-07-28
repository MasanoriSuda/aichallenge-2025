# Task list

- [x] 提出環境ログのSafetyBrake / ShiftOut / Recovery時系列を確認する。
- [x] 現行のtarget / side / corridor ownershipとentry/execute不整合を確認する。
- [x] ROS 2・評価interfaceへの影響がない設計にする。
- [x] SafetyBrakeでPass Missionを`FollowPrepare`へpauseする。
- [x] pause中のownership保持と再検証後resumeを実装する。
- [x] 通常Recovery完了後も継続対象のPass Missionを保持する。
- [x] completion guardを新規entry専用にする。
- [x] candidate corridor内のpreflight goalを実行lineへ伝播する。
- [x] stopped local pathへmoving blockerとalready-in-corridor起動を反映する。
- [x] low-speed candidateだけでは既存missionを消さず、direct実行中のlive corridor lossで停止する。
- [x] pure coreの回帰テストを追加する。
- [x] `docs/spec/mpc-integration.md`へ現仕様を反映する。
- [x] package testとbuildを実行する。
- [x] 最終diffをcode-reviewer観点で確認する。

## Validation

- `colcon build --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 676 tests、error / failure / skip なし
- `git diff --check`: 成功
- ROS 2 topic / service / message / launch / result schema: 変更なし
- `make dev2`: 実走効果確認として未実施
