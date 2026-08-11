# Tasklist

- [x] 最新走行ログと現行HEADを照合する
- [x] 失速と候補全滅の直接条件を特定する
- [x] latched forward escapeのpure判定を追加する
- [x] 横速度を含む到達可能横位置へ変更する
- [x] Returnを実行経路としてwall/到達性検証する
- [x] 最大ShiftOut距離パラメータと長距離候補を追加する
- [x] core単体テストを追加・実行する
- [x] `make autoware-build` を実行する
- [ ] 動的確認項目を記録する

## 動的確認項目

- `latched forward escape` 開始後、rear-clear前に `v_ref` が前車速度へ戻らないこと
- Pass再計画後に `current_ey` が壁方向へ発散しないこと
- 停止車を15〜25 m前方で検出したとき `rollout_lateral_rejected=全候補` にならず、長距離ShiftOut候補が成立すること
- wall contact、EmergencyBrake、solver failureでは従来どおり停止・Recoveryへ移ること

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功、package集計0 failure
- 追加した2境界テストを最終ビルド成果物で再実行: 2/2成功
- `colcon test-result --verbose` は過去成果物 `build/joycon_contract_guard/package.xml` の欠損警告を出したが、集計は1011 tests / 0 errors / 0 failures
