# Tasklist

- [x] `20260817-082938` のwall warningと追い越し遷移を確認する
- [x] full Mission center contractionの不成立理由を特定する
- [x] 短区間wall-escape prefix preflightを実装する
- [x] prefix不成立時のMission handoffを実装する
- [x] core unit testを追加する
- [x] package test/buildを実行する
- [x] 差分をレビューしてコミットする

## Static verification

- [x] `git diff --check`: passed
- [x] `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 targets passed
- [x] `colcon test-result --verbose`: 1240 tests, 0 errors, 0 failures, 0 skipped
  - `build/joycon_contract_guard/package.xml`の既存stale artifact warningは今回の対象外。
- [x] `make autoware-build`: 25 packages built successfully

## Dynamic verification

- [ ] `runtime wall escape prefix accepted`が必要時に発生する
- [ ] prefix採用後もPassの前進速度とtarget lockが維持される
- [ ] prefix不成立時に`holding=current-side`へ残らない
- [ ] `Pass -> Return -> Idle`完遂率が増える
- [ ] crash / wall penaltyが増えない
