# Tasklist

- [x] `20260815-191724` と現行コードの未接続点を確認
- [x] runtime completion replan pending stateを追加
- [x] pure tactical re-arm resolverを追加
- [x] pending中のstale hold/leaseをshadow選考から除外
- [x] same/cross-side atomic replacementへre-arm権限を渡す
- [x] phase/成功時のpending解除を追加
- [x] unit testを追加・実行
- [x] package build/testを実行
- [x] ユーザー変更を除外してコミット

## Definition of Done

- 同一infeasible episodeで再計画要求ログを連打しない。
- targetが前方ならpending中に左右shadow評価が再開する。
- fresh Mission採用時のみ既存Missionが置換される。
- side-by-side/hard fault時はcross-side re-armしない。
- 既存テストを含む対象package testが成功する。

## Validation

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 636/636成功
- `ctest --test-dir /aichallenge/workspace/build/multi_purpose_mpc_ros`: 25/25成功
