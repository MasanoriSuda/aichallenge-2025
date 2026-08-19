# Tasklist

- [x] 最新runで未発動原因を特定する。
- [x] scoped Follow planner admission resolverを追加する。
- [x] BehaviorのFollow planner admissionへ接続する。
- [x] planner requested/runtime authorityログを追加する。
- [x] configコメントを実際のownershipへ合わせる。
- [x] resolver単体テストを追加する。
- [x] package buildとfocused testを実行する。
- [x] 意図したファイルだけをコミットする。

## Definition of Done

- dynamic target activeのFollowでGapPlannerが実行される。
- generic Follow plannerはfalseのまま。
- hard gateではplanner requestもFollow cap suppressionも発生しない。
- ユーザー変更とrun artifactをコミットしない。

## Verification result

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 1347 tests、0 errors、0 failures
- `v2x_follow_gap_planner_enabled`: local/cloudともfalseを維持
- 次回実走確認: `requested=1`、可行回廊がある場面では`active=1`
