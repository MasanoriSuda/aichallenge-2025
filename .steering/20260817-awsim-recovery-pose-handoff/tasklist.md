# Tasklist

- [x] 現行の AWSIM wait / Recovery transition / pose anchor を確認する
- [x] 要件と設計を記録する
- [x] pose handoff 判定と姿勢整合 gate を実装する
- [x] controller の再対応・局所履歴再初期化を実装する
- [x] 単体テストを追加する
- [x] package test を実行する
- [x] `make autoware-build` を実行する
- [x] 今回の変更だけをコミットする

## Verification

- `make autoware-build`: 25 packages finished
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 targets passed
- `colcon test-result --verbose`: 1264 tests, 0 errors, 0 failures
  - unrelated stale `build/joycon_contract_guard/package.xml` lookup warningあり
