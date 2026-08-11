# Tasklist

- [x] 最新ログの全 `Return -> Pass` と直前遷移を照合する
- [x] Return目的を失う再帰更新を原因として特定する
- [x] Return ownershipを状態へ追加する
- [x] core再捕捉policyへownershipを接続する
- [x] tactical revalidation Returnを完遂ownershipにする
- [x] core単体テストを追加・実行する
- [x] 対象packageをビルドする
- [x] 動的確認項目を記録する

## 動的確認項目

- `SafeSeparation target clear ahead; speed-preserving Return` の後、同一周期およびReturn中に
  `same target reacquired during early return` が出ないこと
- Return中の速度上限がRecovery速度へ落ちないこと
- Return完了後は最新geometryから新しいMissionを開始できること
- rear-clear前に別のReturn cancellation条件が成立する既存ケースを壊していないこと

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功
- `git diff --check`: 成功
- 動的試走は未実施。上記4項目を次の`make dev2`で確認する。
