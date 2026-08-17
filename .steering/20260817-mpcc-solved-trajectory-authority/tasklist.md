# Tasklist

- [x] 現HEADのsolver/Mission処理順と失敗経路を確認
- [x] 要件・設計・DoDを記録
- [x] MPCC解軌道抽出helperと単体テストを追加
- [x] MpcProblemへ実行contextとstage境界を保持
- [x] solve成功時のsnapshot保存とcontext/age判定を追加
- [x] 解軌道をreceding-horizon warm-startへ接続
- [x] 物理再検証済み解軌道でsoft wall warningを抑制
- [x] telemetryを追加
- [x] package build/test
- [x] 今回分だけcommit

## Verification

- `docker compose run -T --rm --no-deps autoware-build`: 成功（25 packages）
- `colcon build --packages-select multi_purpose_mpc_ros --symlink-install`: 成功
- `ctest -R "^test_mpcc_progress$" --output-on-failure`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 1242 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
