# Tasklist

- [x] `output/20260817-103103`のwall prefix採用・失効経路を特定する
- [x] Return長期残留時のphase goalを特定する
- [x] wall-escape prefix execution状態と専用terminal distanceを追加する
- [x] Pass entry physical gateとのauthority競合を解消する
- [x] Return/Recoveryのphase-aware goalを修正する
- [x] core unit testを追加する
- [x] package test/buildを実行する
- [x] 差分をレビューしてコミットする

## Static verification

- [x] `git diff --check`: passed
- [x] `make autoware-build`: 25 packages built successfully
- [x] `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 targets passed
- [x] `colcon test-result --verbose`: 1244 tests, 0 errors, 0 failures, 0 skipped
  - `build/joycon_contract_guard/package.xml`の既存stale artifact warningは今回の対象外。

## Dynamic verification

- [ ] `runtime wall escape prefix accepted`後、DP authorityがprefix終端付近まで維持される
- [ ] 採用済みprefixが`Pass entry physical gate held`で即時凍結されない
- [ ] warning継続時もhard guardまたはprefix終端まで安全に実行される
- [ ] `Pass -> Return -> Idle`が成立し、Returnが1周近く残留しない
- [ ] wall/crash penaltyが増えない
