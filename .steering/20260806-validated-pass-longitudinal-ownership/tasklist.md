# Tasklist

- [x] 最新runの非衝突失速点と既存ownership条件を照合する
- [x] requirements/designを記録する
- [x] validated entryの即時handoffと速度ownershipを実装する
- [x] committed ShiftOutのfront-danger抑制へdeadline成立を接続する
- [x] Pass current-overlap確認時間を調整する
- [x] target-clear時のRecovery直行とcoordinated-stop Reverse競合を解消する
- [x] unit testを追加・更新する
- [x] build/testを実行する
- [x] 動的確認項目を記録する
- [ ] `make dev2` で動的効果を確認する（ユーザー実施）

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core|test_stuck_recovery_core`: 873 tests、0 errors、0 failures
- `colcon test-result` は別packageの古い `build/joycon_contract_guard/package.xml` 欠損を警告するが、今回対象のtest結果は成功

## 動的確認項目

- `ShiftOut/Pass/Return static mission validated / front risk brake` が消えること
- validated entry が同周期で `final=Overtake` になること
- committed ShiftOut/Pass中の `Overtake -> SafetyBrake` 回数
- `current body footprints overlap` が0.30秒未満の揺れでcap再適用されないこと
- sustained overlap、壁接触、solver failureでは従来の停止・Recoveryが働くこと
