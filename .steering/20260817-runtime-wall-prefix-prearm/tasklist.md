# Tasklist

- [x] 最新2走のwall-prefix不成立を集計する
- [x] 警告TTC、待機時間、固定prefix距離の不整合を特定する
- [x] path-aware wall forecastを実装する
- [x] TTC制約付きprefix horizonを実装する
- [x] wall-prefixを警告から1制御周期後に評価する
- [x] core unit testを追加する
- [x] package test/buildを実行する
- [x] 差分をレビューしてコミットする

## Static verification

- [x] `git diff --check`: passed
- [x] `make autoware-build`: 25 packages built successfully
- [x] `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 targets passed
- [x] `colcon test-result --verbose`: 1243 tests, 0 errors, 0 failures, 0 skipped
  - `build/joycon_contract_guard/package.xml`の既存stale artifact warningは今回の対象外。
- [ ] `pre-commit`: host・dev containerともcommand未導入のため実行不能

## Dynamic verification

- [ ] `runtime wall escape prefix accepted`が必要時に発生する
- [ ] warningからprefix判定までのelapsedが1制御周期以内になる
- [ ] prefix採用後もtarget、side、front-cap releaseが維持される
- [ ] `holding=current-side`が再発しない
- [ ] `Pass -> Return -> Idle`完遂率が増える
- [ ] wall/crash penaltyが増えない
