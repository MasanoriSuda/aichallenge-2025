# Tasklist

- [x] forced-side transition 状態機械を gateway / crossing / certified に分離
- [x] GapPlanner に複数ステージ接続証明を実装
- [x] 候補・tracking の決定ログを拡張
- [x] 単体テストを更新・追加
- [x] package build / test
- [x] 差分レビューとコミット

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32 test targets 成功、1484 tests、失敗0
- 最終局所再試験: `test_v2x_overtake_core` / `test_overtake_decision_trace` 成功

## 実走で確認するログ

- alternate 採用は `side_transition=1/1/1/1/1` かつ `side_transition_reason=connected-crossing-certified` であること
- 拒否時は `forced-side-transition-disconnected` または `forced-side-transition-uncertified` で位置が記録されること
- tracking failure の `committed_branch` が直前の採用 branch と一致すること
