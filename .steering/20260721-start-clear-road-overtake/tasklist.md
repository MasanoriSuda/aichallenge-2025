# Tasklist

- [x] 最新dev3ログで先頭車の誤Followを特定する。
- [x] ShiftOut解除までのgap欠落時間を計測する。
- [x] rear-side-only候補をCruiseへ分離する。
- [x] active gap-loss holdを実測値に合わせる。
- [x] 単体テストと正本仕様を更新する。
- [x] 対象packageをbuildし、V2X単体テストを実行する。
- [ ] dev3で先頭車のCruise維持とWP61-63の追越し継続を確認する。

## Verification

- `make autoware-build`: 25 packages successful。
- `test_v2x_overtake_core`: 121 tests、failure 0。
- package全体: 523 tests中522成功。既知の
  `PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`のみ失敗。
  現行CSVが閉路終点の重複行を持たないことによる既存不整合で、今回のV2X変更外。
- `git diff --check`: 成功。
