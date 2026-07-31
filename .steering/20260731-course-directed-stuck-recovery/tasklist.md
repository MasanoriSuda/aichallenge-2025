# Tasklist

- [x] 最新ログから競技中止までの Recovery 遷移を特定する
- [x] 既存の横偏差復帰閾値と Reverse-only 条件を確認する
- [x] rollout の course-progress 判定を純粋関数として追加する
- [x] Recovery 候補選択へ course-progress guard を統合する
- [x] 障害物由来 Reverse-first に限定して安全な前進復帰を許可する
- [x] 単体テストを追加・実行する
- [x] `make autoware-build` を実行する
- [x] 変更差分をレビューする

## Definition of Done

- 事象相当の `e_y=-3.63 m` からさらに外へ離れる候補が棄却される。
- 同じ位置から中心へ近づく候補は選択可能である。
- 中心付近と厳密な Reverse-only 条件の従来動作を維持する。
- package test とビルドが成功する。

## 実行結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- 対象 test-result: 675 tests, 0 errors, 0 failures, 0 skipped
- 実走確認: 未実施。`make dev2` でユーザー確認予定。
