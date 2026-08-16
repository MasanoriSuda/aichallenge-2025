# Tasklist

- [x] 最新走行でexecution envelope境界と後段validatorの不整合を確認する
- [x] steering requirements/designを作成する
- [x] execution envelope pure policyへ計画余裕率を追加する
- [x] config・起動ログ・pendingログを更新する
- [x] 単体テストを追加する
- [x] build / testを実行する
- [x] 変更をコミットする

## 検証結果

- `make autoware-build`: 25 packages successful
- execution envelope関連gtest: 3/3 passed
- `test_v2x_overtake_core`: 656/656 passed
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets、1198 tests、0 failures
