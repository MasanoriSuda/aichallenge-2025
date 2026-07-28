# Task List

- [x] 提出rosbagで制御指令あり・実速度なしを確認する。
- [x] 評価側control mode requestが一発送信でackなしと確認する。
- [x] 参加者インターフェース契約を確認する。
- [x] 再送guardを実装する。
- [x] MPC controllerへpublisherと発進待ちRecovery抑止を統合する。
- [x] 単体テストを追加する。
- [x] 正本ドキュメントを更新する。
- [x] 対象テストとビルドを実行する。
- [x] インターフェース互換性を最終確認する。

## 検証結果

- `make autoware-build`: 成功（25 packages）。
- `test_awsim_control_mode_guard`: 8 tests passed。
- `test_start_grid_grace`: 33 tests passed。
- `test_stuck_recovery_core`: 98 tests passed。
- `/awsim/control_mode_request_topic`は既存名・型を維持し、Domain 0、
  `/admin/awsim/*`、`aichallenge_system/`は変更していない。
- 提出環境実走は未実施。次回ログでReady/Start要求、再送、motion confirmedを確認する。
