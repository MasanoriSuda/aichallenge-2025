# Tasklist

- [x] 現行の`FollowPrepare`遷移を確認する
- [x] 要件と設計を記録する
- [x] pause originを状態へ追加する
- [x] terminal action resolverをpure coreへ追加する
- [x] controllerの分散判定をresolver利用へ置換する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 差分と無挙動変更を確認する

## Definition of Done

- 既存のtime/distance expiry優先順位が維持される
- target異常時は従来どおりRecoveryへ入る
- rear-clear時は従来どおりReturnへ入る
- transient pauseは従来どおりHoldする
- 速度・経路・安全marginの設定差分がない

## 動的確認項目

次の性能修正前に実走ログで確認する。

- `FollowPrepare`ログにoriginが出る
- SafetyBrake由来pauseのoriginが`ShiftOut`または`Pass`
- Recovery完了由来pauseのoriginが`Recovery`
- timeout、Recovery、Returnの発生回数が修正前と不自然に変わらない

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test suites成功
- `colcon test-result`: 961 tests、0 errors、0 failures、0 skipped
- param / launch / topic / serviceの変更なし
