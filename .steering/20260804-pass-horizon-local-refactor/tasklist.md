# Task list

- [x] 現行 Pass horizon と SafeSeparation の依存関係を確認する
- [x] 要件と設計を記録する
- [x] full-path preflight 呼び出しを抽出する
- [x] Pass horizon decision request 構築を抽出する
- [x] SafeSeparation 開始処理を抽出する
- [x] 短期安全判定を共有する
- [x] build/test を実行する
- [x] 差分を確認し、挙動変更がないことをレビューする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 833 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 問題なし
