# Tasklist

- [x] 最新dev3ログから再現条件を特定する。
- [x] 停止車両回避と通常追い越しの所有権切れを特定する。
- [x] 共通コース回廊の停止車両候補を実装する。
- [x] 近距離停止車両入口を3.0 mへ変更する。
- [x] `OvertakeLine`所有権判定を修正する。
- [x] gapが無い場合のFollow減速とスタート猶予を維持する。
- [x] レース開始前のV2X状態・stall watchdog進行を抑止する。
- [x] 単体テストを追加する。
- [x] 正本仕様を更新する。
- [x] 対象テストとAutowareビルドを実行する。

## Verification

- `make autoware-build`: 25 packages successful。
- `test_v2x_overtake_core`: 120 tests、failure 0。
- package全体テスト: 517 tests中516成功。既存trajectoryの閉路端点前提と現行CSVが
  一致しない`PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`のみ失敗。
- `git diff --check`: 成功。
