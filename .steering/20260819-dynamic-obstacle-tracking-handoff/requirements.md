# Requirements

## 目的

Follow 中の動的障害物回避候補を追従 MPC へ渡した直後に solver failure と解除・再採用を反復する不具合を止める。

## 実走根拠

- 対象: `output/20260819-181127`
- dynamic lateral escape active は 200 回、その直後の solver failure は 179 回。
- MPC solver failure は 211 回（直前 run は 63 回）。
- 横目標は約 `-2.53 .. +3.51 m`、左右切替は 11 回。
- 正常な `Pass -> Return -> Idle` 完遂は 0 回。

## 要求

1. GapPlanner の幾何的 corridor を、現在の横位置・横速度・横加速度上限から到達可能か検証してから MPC へ渡す。
2. 最初の tracking MPC solve が成功するまでは、前車追従速度制限を解除しない。
3. 新しい lateral escape が solver failure を起こした場合、同一方式の即時再採用を短時間止める。
4. authority が成立しなかった scoped Follow planner 出力を、下流の MPC bounds/reference へ流さない。
5. Overtake Mission、LowSpeedAvoidance、EmergencyBrake、既存 ROS 2 interface は変更しない。

## 制約

- 参加者実装内に閉じる。
- `output/`、result JSON、crash blob は変更・コミットしない。
- 実走の最終効果確認はユーザー環境で行う。
