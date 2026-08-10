# Tasklist

- [x] 現行のcache更新・Mission置換経路を確認する
- [x] hard/soft cache policyを実装する
- [x] ego/target移動量freshnessを実装する
- [x] Mission置換をtransactionalにする
- [x] hard fault時のcache拒否とログを追加する
- [x] unit testを追加する
- [x] package build/testを実行する
- [ ] 動的確認項目を記録する

## Definition of Done

- soft miss 1周期でcacheが失われない
- stale/moved/hard-fault candidateは再利用されない
- replacement失敗時に旧Mission generation/状態が維持される
- `multi_purpose_mpc_ros` のbuild/testが成功する

## Static verification

- `make autoware-build`: 成功（25 packages）
- `colcon build --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets、937 tests、失敗0

## Dynamic verification checklist

- `last-feasible ... cache retained across soft miss` の後にrescueが成立するか
- `replacement prepare/commit rejected` 時にgenerationが変わらないか
- hard fault時に `cache cleared` となり再利用されないか
- `Pass -> Return -> Idle` 完遂数が増え、SafeSeparation/Recoveryが減るか
