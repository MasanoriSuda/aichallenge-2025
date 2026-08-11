# Tasklist

- [x] 20260811-141846の固着ログを特定する
- [x] Drive要求が抑止される状態遷移を特定する
- [x] pending中のDrive再要求仲裁をpure coreへ追加する
- [x] controllerから古い最終指令による抑止を撤去する
- [x] 再現単体テストを追加する
- [x] `make autoware-build` を実行する
- [x] package単体テストを実行する

## 動的確認項目

- `gear=Reverse, speed≈0` 後に `Stuck recovery gear requested: gear=Drive` が出ること
- Drive報告欠落時にも、0.25秒以上空けてDrive要求が再送されること
- freshなDrive報告後にRecoveryを抜け、Overtakeまたは通常前進へ戻ること
- `stopping for forward Overtake handoff` が数十秒継続しないこと

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_stuck_recovery_core`: 126/126成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test target成功
- `colcon test-result`: 1012 tests / 0 errors / 0 failures
- 既存の `build/joycon_contract_guard/package.xml` 欠損警告は出るが、今回の変更とは無関係
