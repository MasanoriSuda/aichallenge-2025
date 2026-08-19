# Tasklist

- [x] 現行ログと採否経路を確認する
- [x] interface契約を確認する
- [x] Decision Traceモジュールを追加する
- [x] authority拒否理由を構造化する
- [x] controllerへprimary / alternate / authority traceを接続する
- [x] tracking失敗・復帰ログを統一する
- [x] 単体テストを追加する
- [x] package build / testを実行する
- [x] 差分をレビューしてコミットする

## Verification

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 29 targets、1383 tests、失敗0
- 最終差分の`test_overtake_decision_trace` / `test_v2x_overtake_core`: 失敗0
- `git diff --check`: 問題なし
- `pre-commit`: ホストに実行ファイルがなく未実行
