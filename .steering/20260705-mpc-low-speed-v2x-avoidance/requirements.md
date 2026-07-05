# MPC Low Speed V2X Avoidance Requirements

作成日: 2026-07-05
状態: Draft

## 目的

近距離に停止または低速の V2X 車両がいる状態から開始する gate2 相当のケース、およびレース中に競争停止車両が発生したケースで、即 SafetyBrake に落ち続けず、低速で横へかわす状態を追加する。

## 要求

- `/v2x/vehicle_positions` の topic 名と message 型は変更しない。
- 既存の Cruise / Follow / Overtake / SafetyBrake の挙動を大きく変えない。
- 前方距離が短くても、前車が停止または低速で、MPC horizon 内に通過可能 gap がある場合は低速回避を選べる。
- 低速回避中は gap planner を許可し、`gap_target_bias` による横方向目標を使えるようにする。
- 低速回避中は通過側を固定または自動ロックできる。
- 低速回避中は、実制御時だけ手前の horizon 点にも通過側へ寄せるソフト参照を入れる。
- 低速回避中の速度上限は config で指定できる。
- 通過可能 gap がない場合は SafetyBrake に倒す。

## Config

```yaml
mpc:
  v2x_low_speed_avoidance_enabled: true
  v2x_low_speed_avoidance_distance: 8.0
  v2x_low_speed_avoidance_velocity: 2.0
  v2x_low_speed_avoidance_max_front_speed: 1.0 # 予備設定。開始条件の hard gate にはしない。
  v2x_low_speed_avoidance_min_gap_width: 0.5
  v2x_low_speed_avoidance_clear_distance: 8.0
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
```

## 受け入れ条件

- `make autoware-build` が成功する。
- gate2 のように前車が約 7m 先にいる状態で `SafetyBrake` へ即落ちする前に `LowSpeedAvoidance` を選択できる。
- `LowSpeedAvoidance` 中に近傍 V2X 車両が残っている間は、3台目横で通常速度へ戻らず徐行を維持できる。
- gap がない場合は従来通り停止側へ倒せる。
