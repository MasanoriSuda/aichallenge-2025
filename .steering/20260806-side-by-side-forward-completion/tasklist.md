# Tasklist

- [x] 最新ログから並走後の Recovery / SafetyBrake 経路を再現する
- [x] 既存 forward escape、overlap confirmation、Pass horizon の優先順位を確認する
- [x] 並走完遂 pure policy を追加する
- [x] predicted-overlap 横再計画より前方完遂を優先する
- [x] current-overlap grace を SafeSeparation と共有する
- [x] forward escape 中の絶対上限処理を局所枠へ接続する
- [x] rear-clear 前の早期 Return を抑止する
- [x] 回帰試験と設計ドキュメントを更新する
- [x] package test、build、diff check を実行する

## Verification

- `make autoware-build`: 成功（25 packages、`multi_purpose_mpc_ros` を含む）
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core --output-on-failure`: 成功
- `colcon test-result --verbose`: 873 tests、0 errors、0 failures（既存の欠損した `joycon_contract_guard/package.xml` に対する集計警告あり）
- `git diff --check`: 成功
- `make dev2`: 未実施。次回試走で `forward_commit=1`、`side_by_side_commit`、rear-clear後Returnを確認する
