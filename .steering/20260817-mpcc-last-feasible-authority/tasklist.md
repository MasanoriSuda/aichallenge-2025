# Tasklist

- [x] 最新試走のauthority遷移を確認
- [x] 現行の抽出・保存・再検証処理を確認
- [x] 要件・設計を記録
- [x] 抽出失敗診断とOSQP residual許容を追加
- [x] Mission累積進捗によるphase handoffを実装
- [x] last-feasible再検証と実行authority橋渡しを実装
- [x] 単体テストを追加
- [x] package build/test
- [x] 今回分だけcommit

## Verification

- `docker compose run -T --rm --no-deps autoware-build`: 成功（25 packages）
- `colcon build --packages-select multi_purpose_mpc_ros --symlink-install`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros/test_results --verbose`: 1215 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
