# Tasklist

- [x] 基準走行のShiftOut時間と完遂数を整理
- [x] 変更対象と固定条件を定義
- [x] 4設定をCandidate値へ変更
- [x] 差分確認
- [x] `make autoware-build`
- [x] 次回試走の判定項目を整理

## 検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- ROS topic/service、壁判定、Pass未latch速度、SafetyBrakeの変更なし

## 次回試走の判定項目

- 新規`Idle -> ShiftOut`の`front_distance`が6 m以上になっていること。
- ShiftOut中の`closing`が距離余裕に応じて1.5～2.0 m/sへ上がること。
- `Idle/FollowPrepare -> ShiftOut`から`ShiftOut -> Pass`までを計測すること。
- 基準の1.8～3.2秒から、1.5～2.0秒程度へ短縮すること。
- `Pass -> Return`、Recovery理由、SafetyBrake中断回数を基準走行と比較すること。
- 新規ShiftOut回数が大きく減る場合はCandidate不採用とすること。
