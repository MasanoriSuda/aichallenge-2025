# Tasklist

- [x] 最新ログとSafeSeparation／ContactContinuation経路を照合する
- [x] target後方化だけでは車体rear-clearでないことを確認する
- [x] rearward contact completion policyをcoreへ追加する
- [x] controllerのPass latch／commit stageへ接続する
- [x] dev／cloud設定へ専用上限を追加する
- [x] core単体テストを追加・実行する
- [x] 対象packageをビルドする
- [x] 動的確認項目を記録する

## 動的確認項目

- 進捗中の `ContactContinuation` が0.8秒で終了せず、rearward tailへ移ること
- `target_s < 0`かつ車体非重複後に `Pass -> Return -> Idle` へ進むこと
- 同じ事象で `SafeSeparation aborted: short horizon unsafe` が出ないこと
- 壁接触、正面衝突、進捗停止ではrearward tailが有効にならないこと
- 接触／Recoveryと異常ラップ時間が増えていないこと

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功
- `git diff --check`: 成功
- 動的試走は未実施。上記5項目を次の`make dev2`で確認する。
