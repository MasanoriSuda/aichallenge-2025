# Tasklist

- [x] planning/tracking trace に資格確認状態を追加
- [x] 未資格候補失敗時の last-feasible hold を実装
- [x] 資格確認成功・棄却を target/side/branch 単位で記録
- [x] 単体テストを追加・更新
- [x] package build / test
- [x] 差分レビュー
- [x] コミット

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（32/32 targets）
- `colcon test-result --verbose`: 1489 tests、0 errors、0 failures
- `git diff --check`: 問題なし

動的効果確認は未実施。次回試走では planning の
`qualification-pending-*` と tracking の `qualified` / `qualification-rejected`、
および `qualification_hold` を照合する。
