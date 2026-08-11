# Tasklist

- [x] 最新実走と現行コードの失敗経路を照合する
- [x] 設計・制約を記録する
- [x] dynamic Mission waitのinvalidated-generation policyを修正する
- [x] rear-clear rolloutへFrenet course progress係数を追加する
- [x] unit testを追加する
- [x] build/testを実行する
- [x] 動的確認項目を記録する

## Definition of Done

- `FollowPrepare -> Recovery, reason=current Mission generation invalidated` がwait開始直後に発生しない
- outer pathのrear-clear予測が物理距離をcourse距離と誤認しない
- hard faultのfail-closed条件を維持する
- `multi_purpose_mpc_ros` のbuild/testが成功する

## Dynamic verification checklist

- `dynamic Mission wait entered` からIdle/replacementまでの時間
- `dynamic Mission wait failed: current Mission generation invalidated` の回数（期待値0）
- entry時の `rear_clear_s` とruntime `required_forward` の乖離
- `outer pass becomes inside before rear-clear` の回数
- `Pass -> Return -> Idle` 完遂率
- SafetyBrake / Recovery / wall contact回数

## Static verification result

- `git diff --check`: pass
- `make autoware-build`: pass（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: pass（25/25 tests）
- `colcon test-result --verbose`: 994 tests, 0 errors, 0 failures, 0 skipped
- 既存build treeの`joycon_contract_guard/package.xml`欠落warningは出たが、test resultは成功
