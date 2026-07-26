# Tasklist

- [x] 現行のRecovery履歴リセット箇所を確認
- [x] 倍増条件・リセット条件・上限を設計
- [x] adaptive retry設定を追加
- [x] retry level trackerを実装
- [x] rollout・制動・escape確認へactive targetを適用
- [x] session resetとRejoinCompleteへtrackerを連結
- [x] 単体テストを追加
- [x] 対象packageをビルド・テスト
- [x] 実走確認項目を記録

## Definition of Done

- 前進5 m未満でRecoveryが再発すると後退目標が2倍になる。
- 目標は4 mを超えない。
- 5 m以上正常前進すると次回目標は初期値へ戻る。
- 延長距離について静的壁・V2X安全判定を省略しない。
- `multi_purpose_mpc_ros` のビルド・テストが成功する。

## Verification

- `make autoware-build`: 25 packages succeeded
- `colcon test --packages-select multi_purpose_mpc_ros`: 630 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: passed

## 実走確認項目

1. 起動ログで `adaptive_retry=enabled/x2.00/max=4.00 m/reset=5.00 m` を確認する。
2. 初回のfast rejoinでは `adaptive_retry=0`、`escape_target=0.800 m` を確認する。
3. 5 m進む前の再Recoveryで
   `Stuck recovery repeated after short rejoin: retry_level=1` を確認する。
4. 再試行の目標が `1.600 m`、次回が `3.200 m`、以後は `4.000 m` で止まることを確認する。
5. fast rejoinが成立しない場合は通常目標が `2.000 -> 4.000 m` になることを確認する。
6. Normalで5 m以上前進した後に
   `Stuck recovery adaptive Reverse reset after 5.00 m` が出ることを確認する。
7. リセット後の次回Recoveryが `adaptive_retry=0` と初回目標へ戻ることを確認する。
8. 延長された後方rolloutが壁またはV2Xで棄却された場合、Reverseを開始しないことを確認する。
