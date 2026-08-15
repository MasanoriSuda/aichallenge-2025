# Tasklist

- [x] `20260815-200955` のチャタリング経路を特定
- [x] terminal DynamicMissionWait contractを追加
- [x] strict same-side prefix re-armを接続
- [x] fresh search handoffを接続
- [x] unit testを追加
- [x] build/package testを実行
- [x] ユーザー変更を除外してコミット

## Definition of Done

- terminal abortで未rearmの同側candidateを再採用しない。
- strict re-arm済み同側candidateとcomplete alternateは採用できる。
- 候補なしなら評価完了後にfresh searchへ戻る。
- 通常DynamicMissionWaitの既存テストを維持する。

## Validation

- `make autoware-build`: 成功（25 packages）
- terminal waitと関連MPCC-lite単体テスト: 3/3成功
- `ctest --test-dir /aichallenge/workspace/build/multi_purpose_mpc_ros`: 25/25成功
- `git diff --check`: 成功
