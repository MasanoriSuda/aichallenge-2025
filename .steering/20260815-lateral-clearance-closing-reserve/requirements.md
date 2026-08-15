# Requirements

## 目的

`output/20260815-173724`で確認した、横離隔が成立する前に前車へ詰めすぎ、
SafetyBrakeとOvertakeを反復する現象を減らす。

最新走行ではSafetyBrake進入78回のうち59回がOvertake中で、55回が
`moving front inside hard center distance`だった。正常な`Pass -> Return`は
34 episode中2回だった。

## 課題

- `v2x_follow_distance: 5.0`は追従対象範囲であり、実際の速度追従目標は
  中心間3.0 mになっている。
- 2 m車体同士では中心間3.0 mは面間約1.0 m、SafetyBrakeの中心間
  2.05 mは面間約0.05 mである。
- 既存の2.30 m closing reserveは、車体矩形が既に非分離になってからしか
  有効にならず、接触前の保護として遅すぎる。
- Missionの予測body-clear時間とplanned closing speedを、entry時の必要車間へ
  明示的に反映していない。

## 制約

- 横離隔成立後のPass加速は抑制しない。
- 壁、車体、EmergencyBrakeのhard guardは緩和しない。
- start-grid breakoutの専用処理には通常走行用の新entry距離を強制しない。
- ROS topic/service、評価schema、`aichallenge_system`は変更しない。

## Definition of Done

- 前車が前方にあり横方向の車体離隔が未成立なら、縦方向にまだ離れていても
  closing reserveが作動する。
- 横方向の車体離隔成立後はclosing reserveを解除する。
- 通常の新規Mission entryは、予測body-clear時間に応じた必要中心間距離を満たす。
- Follow時は中心間4.0 mを目標にし、4.5 mから連続的に速度を合わせる。
- 単体テストと`make autoware-build`が成功する。
