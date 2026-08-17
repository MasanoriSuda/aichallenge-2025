# Tasklist

- [x] 最新runのPass/Return失敗経路を確認する
- [x] Returnで切れているMPCC/receding-horizon条件を特定する
- [x] phase分類を局所helperへ整理する
- [x] Returnをreceding-horizonとsolved trajectory contextへ追加する
- [x] rear-clear後のtarget bounds解除をReturnへ拡張する
- [x] 単体テストを追加する
- [x] package test/buildを実行する
- [x] 差分をレビューしてコミットする

## Definition of Done

- `multi_purpose_mpc_ros` の単体テストが全件成功する
- `multi_purpose_mpc_ros` がDocker内でbuildできる
- topic/service/interface契約に変更がない
- `config.yaml` とresult生成物の既存変更をコミットへ含めない

## Verification

- `make autoware-build`: 25 packages successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 28 test targets successful
- `colcon test-result --verbose`: 1264 tests, 0 errors, 0 failures, 0 skipped
- 動的な `Return -> Idle` 成功率は次回 `make dev2` 走行で確認する
