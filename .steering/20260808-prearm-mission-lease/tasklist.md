# Tasklist

- [x] 最新1周runと直前成功runを比較する
- [x] requirements/designを記録する
- [x] target-scoped pre-arm validation leaseをpure resolverで実装する
- [x] pre-armのidentityからside/closing speedを外す
- [x] latest validated Missionだけをhandoffする
- [x] full Missionによるcompletion admission overrideを追加する
- [x] ログとconfigを追加する
- [x] core unit testを追加する
- [x] `docs/spec/mpc-integration.md`を更新する
- [x] package testとbuildを実行する
- [x] `git diff --check`とinterface境界を確認する

## 動的確認項目

- 同一targetのside/closing変化で`entry_stable`がゼロへ戻らないこと
- `prearm_lease=1`ではOvertakeLineがIdleで、横goalを出さないこと
- lease中にhard faultが出れば即解除されること
- handoff時のMissionがその周期の`mission candidate selected`と一致すること
- `Idle -> ShiftOut`、`ShiftOut -> Pass`、`Pass -> Return -> Idle`の回数
- candidate reject、pre-arm再始動、Follow cap、Recovery、接触の件数

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 成功（866 tests、0 failures）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 targets）
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  890 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
- `make dev2`: 未実施。効果確認は上記の動的確認項目で行う
