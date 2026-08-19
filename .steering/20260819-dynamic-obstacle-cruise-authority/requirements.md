# Requirements

## Purpose

Mission が成立しないだけで Follow へ落ちて失速する経路を切り離し、通常走行中も
V2X 車両を動的障害物として GapPlanner の横回避経路を実行できるようにする。

## Observed problem

- 全 V2X の接近予測と Follow 中の GapPlanner は既に存在する。
- ただし動的障害物は tactical target へ昇格した後、左右の完全 Mission 生成へ
  渡されるため、Mission admission が失敗すると Follow の速度制限が残る。
- GapPlanner が実行可能な横回避経路を生成した周期でも、縦方向は前車追従となり、
  横へ出る前進力を失うことがある。

## Requirements

1. 動的障害物 authority、Follow 状態、実行可能な GapPlanner 回廊が同時に成立した
   周期は、Mission の有無にかかわらず横回避経路を MPC へ渡す。
2. 横回避が実行可能な周期は通常の Follow / moving-front cap を解除する。
3. EmergencyBrake、solver recovery、無効な回廊、横移動を伴わない経路では解除しない。
4. GapPlanner が不成立なら従来どおり Follow / no-gap limit を適用する。
5. OvertakeLine が既に実行権を持つ場合は、既存 Mission の制御権を奪わない。
6. 壁・V2X の state bounds と GapPlanner の速度制限は従来どおり hard constraint とする。
7. ROS 2 topic / service / message と評価インターフェースを変更しない。

## Out of scope

- OvertakeLine FSM の廃止
- 左右 Mission / Return / no-return の再設計
- 新しい非線形 solver の追加
- Recovery / Reverse の変更
- EmergencyBrake の緩和
