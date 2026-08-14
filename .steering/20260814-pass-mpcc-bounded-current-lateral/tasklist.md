# Tasklist

- [x] 最新走行の失敗経路を整理する
- [x] 横補正admissionの純粋関数と単体テストを追加する
- [x] rolling same-side候補へ現在横位置を追加する
- [x] 大横補正候補をpreflight前に棄却する
- [x] 通常・cloud設定へ上限を追加する
- [x] 対象packageの単体テストとビルドを実行する
- [x] 検証結果と残課題を記録する

## 検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 570 tests、failure/error 0
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets成功

## 試走確認

- 起動ログが `same_side_dy<=0.35 m` となること。
- `fresh same-side PassPlan replaced` の `dy` が0.35 m以下であること。
- 直近失敗相当の約1.5 m補正が採用されず、候補不成立時は
  `rolling_lateral_rejected` が増えること。
- `Pass -> Return -> Idle`、wall Recovery、target overlap、MPCC-lite計算時間を
  前回runと比較すること。

## 残課題

MPCC-lite評価はまだ制御callback内の同期処理である。今回は候補の早期棄却で計算量を
減らした。専用worker化は、controller/model/V2Xをimmutable snapshotへ切り出した後の
別ステアリングで行う。
