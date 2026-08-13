# Requirements

## 背景

`output/20260813-112615/d1/autoware.log` では、前回追加した receding-horizon
Mission continuity により次が改善した。

- `SafeSeparation aborted: invalid input`: 2回から0回
- runtime wall Return直後の `Return -> Pass`: 2回から0回
- receding-horizon由来のsafe trajectory prefix: 7回
- full-speed forward escapeの周期ログ: 3回から12回

一方、episode 10では `target_s=0.57 m` まで前進できていたにもかかわらず、
`SafeSeparation aborted: local distance limit` でPassを破棄した。実行時は
`v_ref=11.11 m/s` のfull-speed forward escapeだったが、rear-clear rolloutは
`v2x_overtake_shiftout_max_closing_speed=2.0 m/s` を使い続け、必要完遂距離を
過大評価していた。

## 目的

1. Pass中のrear-clear rolloutへ、実際に選択可能なfull-speed forward escape速度を渡す。
2. 横軌道の壁・車体制約と縦速度を同じrolloutで評価する。
3. 実行可能な前進中に固定12 m局所上限だけでPassを破棄する回数を減らす。

## 制約

- 40 m / 10 sの絶対Pass上限は変更しない。
- 壁接触、壁サンプル欠落、緊急制動、solver異常、corridor blockは速度couplingを無効化する。
- 車体と予測sweepの分離が確認できない状態にはfull-speed rolloutを適用しない。
- `aichallenge_system`、ROS 2 topic/service契約、評価schemaは変更しない。
- `aichallenge/result-summary.json` のユーザー変更は含めない。

