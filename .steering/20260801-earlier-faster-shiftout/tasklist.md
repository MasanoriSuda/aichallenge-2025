# Tasklist

- [x] 基準走行のShiftOut時間と完遂数を整理
- [x] 変更対象と固定条件を定義
- [x] 4設定をCandidate値へ変更
- [x] 差分確認
- [x] `make autoware-build`
- [x] 次回試走の判定項目を整理
- [x] Candidate AがFollowを長期化させることを確認し不採用化
- [x] 距離設定をBaselineへ復元
- [x] Candidate Bをビルド
- [x] Candidate Bの試走判定項目を整理

## 検証結果

- Candidate Bの`git diff --check`: 成功
- Candidate Bの`make autoware-build`: 成功（25 packages）
- ROS topic/service、壁判定、Pass未latch速度、SafetyBrakeの変更なし

## Candidate A結果

- 6 m条件は早期準備の開始条件ではなく、新規ShiftOutの禁止条件として動作した。
- 通常Followが長期化したため不採用。

## Candidate B試走の判定項目

- ShiftOut中の`closing`が距離余裕に応じて1.5～2.0 m/sへ上がること。
- `Idle/FollowPrepare -> ShiftOut`から`ShiftOut -> Pass`までを計測すること。
- 基準の1.8～3.2秒から、1.5～2.0秒程度へ短縮すること。
- `Pass -> Return`、Recovery理由、SafetyBrake中断回数を基準走行と比較すること。
- Recoveryが同数以上なら相対速度変更も不採用とすること。
