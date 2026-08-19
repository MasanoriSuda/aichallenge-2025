# Requirements

## Purpose

`21d378c` で追加した dynamic lateral escape authority が Follow 中にも
GapPlanner 出力を受け取れるようにし、Mission 不成立時の横回避を実走可能にする。

## Observed failure

- 設定上 dynamic lateral escape authority は有効だった。
- 最新 run `20260819-175416` では Follow debug 42件がすべて `allow_gap=0`。
- `Dynamic-obstacle lateral escape authority: active=1` は0件。
- generic Follow 用 `v2x_follow_gap_planner_enabled` が false のため、authority 判定前に
  GapPlanner 自体が停止していた。

## Requirements

1. dynamic-obstacle target が active な Follow では、generic Follow planner 設定と独立して
   GapPlanner の可行性評価を許可する。
2. soft な curve/overtake forbidden は scoped dynamic planning を妨げない。
3. explicit forbidden waypoint、EmergencyBrake、solver recovery/fallback、pre-arm、既存の
   active Pass ownership では planner を許可しない。
4. planner が feasible かつ有意な横移動を生成した場合だけ、既存authorityがFollow capを
   解除する。
5. generic Follow planner の既定値 false は維持し、全Followへ挙動を広げない。
6. ROS 2・評価インターフェースを変更しない。

## Out of scope

- Mission FSM、Return、Recovery、solver の変更
- 全FollowでのGapPlanner有効化
- EmergencyBrakeや明示的な禁止waypointの緩和
