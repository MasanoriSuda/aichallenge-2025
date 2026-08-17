# Tasklist

- [x] `output/20260817-110115`の壁接触・ShiftOut・Recovery時系列を確認する
- [x] 壁監視が新規Mission開始前に評価済みであることを確認する
- [x] fresh ShiftOut wall entry gateをcoreへ追加する
- [x] controllerのphase遷移・Mission freeze前へ統合する
- [x] direct Pass・既存Missionを除外するunit testを追加する
- [x] package test/buildを実行する
- [x] 差分をレビューしてコミットする

## Static verification

- [x] `git diff --check`: passed
- [x] `make autoware-build`: 25 packages built successfully
- [x] `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 targets passed
- [x] `colcon test-result --verbose`: 1245 tests, 0 errors, 0 failures, 0 skipped
  - `build/joycon_contract_guard/package.xml`の既存stale artifact warningは今回の対象外。

## Dynamic verification

- [ ] `wall=1`またはcurrent wall warning中に`Idle -> ShiftOut`へ入らない
- [ ] ログに`fresh ShiftOut held by current wall state`が出る
- [ ] 壁状態解消後にfresh candidateを再評価し、追い越しへ復帰できる
- [ ] predicted wall warningは従来どおりpreplan/prefix処理へ進む
- [ ] `Pass -> Return -> Idle`の完遂を維持する
