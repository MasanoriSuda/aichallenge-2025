# Tasklist

- [x] 最新走行ログと現行authority/failover経路を照合する
- [x] 変更範囲と非対象を固定する
- [x] attempt IDとmission episodeを分離する
- [x] GapPlanner reject gateを構造化する
- [x] primary失効後のalternate評価を局所リファクタする
- [x] runtime failover traceを追加する
- [x] Decision Trace / core単体テストを追加・更新する
- [x] 対象packageをbuild/testする
- [x] `git diff --check`とインターフェース非変更を確認する
- [x] 検証結果を記録し、ユーザー変更を除外してcommitする

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon build --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 29/29 CTest成功、0 failure
- `colcon test-result --verbose`: 1387 tests、0 errors、0 failures
- 動的効果確認: 次回の`make dev2`試走で実施する
