# Tasklist

- [x] 2本の走行ログから再現条件を確定
- [x] wall admission gateを共通化
- [x] active overtake predictionを10 Hzで監視
- [x] active overtake holdを最終制御sourceへ追加
- [x] DP tracking release confirmationを追加
- [x] authority reasonログを追加
- [x] unit test追加・更新
- [x] package build/test
- [x] 差分レビュー
- [x] コミット

## Verification

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 30 targets、1447 tests、失敗0
- 変更対象の直接再実行:
  - `test_overtake_execution_orchestrator`: 27 tests、失敗0
  - `test_v2x_overtake_core`: 788 tests、失敗0
- `git diff --check`: 問題なし
