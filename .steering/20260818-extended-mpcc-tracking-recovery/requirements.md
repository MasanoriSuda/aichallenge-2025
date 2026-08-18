# Requirements

## 背景

`20260818-190144` では拡張MPCCの数値収束は改善した一方、追従と追い越し完遂が悪化した。
`Idle -> ShiftOut` 9回に対して `Pass -> Return` は0回で、横誤差・姿勢誤差が大きい状態から壁接触とRecoveryへ遷移した。

## 目的

- 拡張MPCCの横位置・姿勢追従を回復する。
- 拡張MPCCと既存3-state MPCCの切替で速度指令を不連続にしない。
- 前方危険、壁、速度上限などのhard constraintは緩和しない。

## 制約

- ローカル進捗座標、warm-start再基準化、failure circuit breakerは維持する。
- `aichallenge_system`、ROS 2 topic/service契約、評価結果schemaは変更しない。
- ユーザー生成の `aichallenge/result-summary.json` は変更対象に含めない。
