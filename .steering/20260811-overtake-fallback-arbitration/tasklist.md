# Tasklist

- [x] 最新ログと現行分岐を照合する
- [x] 要件・設計・安全境界を記録する
- [x] tactical no-return re-arm を core 仲裁器へ追加する
- [x] cross-side Mission commit へ re-arm を伝播する
- [x] speed-preserving soft disengagement 仲裁器を追加する
- [x] controller の SafeSeparation abort 分岐へ統合する
- [x] core 単体テストを追加する
- [x] package build と test を実行する
- [x] 動的確認項目を記録する

## Definition of Done

- 既存 hard-fault fail-closed を保ったまま、物理的に再分離した SafeSeparation から一度だけ alternate Mission を選べる。
- 次善策なしの物理 clear 軟失敗で Recovery 速度制限へ落ちない。
- `test_v2x_overtake_core` が通る。
- `multi_purpose_mpc_ros` がビルドできる。

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）
- 実走は未実施。次回 `make dev2` で以下を確認する。
  - `SafeSeparation tactical alternate reselect` の直後に `last-feasible maneuver rescue accepted` が出ること
  - accepted 時に side が反転し `ShiftOut` へ再遷移すること
  - alternate 不成立の物理 clear 軟失敗で `soft Mission abort preserves speed` が出て Recovery へ入らないこと
  - overlap、wall、EmergencyBrake、solver fault では従来どおり Recovery へ入ること
