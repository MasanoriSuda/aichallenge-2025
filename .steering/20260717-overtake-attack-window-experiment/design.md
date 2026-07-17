# Design: Overtake Attack Window Experiment

## 1. Stage-aware speed reference

現状は`front_speed + v2x_overtake_velocity_advantage`をMPC参照へ`min()`適用するため、
追い越し全期間で前車+1m/sが実質上限になる。

実験では次の2段階にする。

- ShiftOut: 前車+`shiftout_max_closing_speed`と進入速度の大きい方を上限とする。
- Pass: 前車由来の上限を適用せず、軌道CSV、global/domain v_max、動的制約が作る元の参照を使う。

SafetyBrakeや明示的なvelocity limitはこの後段で従来どおり適用される。
設定スイッチでlegacy挙動へ戻せるようにする。

## 2. Hard-curve completion guard

soft curve禁止と、物理的に危険なhairpinを分離する。新規開始時だけ、現在位置から
`completion_hard_curvature`を超える最初のwaypointまでの距離を求める。

推定式:

```
relative_gain = front_distance + return_clear_distance
relative_speed = planned_ego_speed - front_speed
pass_distance = planned_ego_speed * relative_gain / relative_speed
required_distance = max(shift_distance, pass_distance) + merge_buffer
available_distance = distance_to_hard_curve - curve_buffer
```

relative speedが設定下限未満、入力が不正、または`available < required`なら新規開始を拒否する。
既にShiftOut/Passへ入った車両はこのentry guardで中断せず、既存のhard WP、inner curve、
EmergencyBrake、solver Recoveryに従う。

## 3. Path-time V2X prediction

各horizon pointの予測時刻を`(i+1)*Ts`から、segmentごとの
`distance / max(reference_speed, prediction_min_ego_speed)`累積へ変更する。
観測ageは従来どおり別途加算し、`v2x_prediction_time`で上限を設ける。

## 4. Multi-front and curve policy

- `v2x_multi_front_gap_enabled: true`
- `v2x_vehicle_vehicle_gap_enabled: false`

これにより前方2台で即拒否せず、merged occupied intervalの外側のみ候補になる。

soft curve継続は有効にするが、開始距離3m、inner curve禁止、hard WP禁止を維持する。
曲率閾値は0.04、soft lookaheadは6mとし、変更を一度に過大化しない。

## 5. Solver experiment

横目標goalの変化量を0.25m/cycleから0.04m/cycleへ下げ、abort閾値を3周期から8周期へ増やす。
solver failure中は従来の減速fallbackを維持する。同一周期retryは状態更新の再入可能化後に行う。

## Compatibility

- 変更は`aichallenge_submit/multi_purpose_mpc_ros`内に閉じる。
- `/control/command/control_cmd`、`/v2x/vehicle_positions`の名前・型を変更しない。
- Domain 0/1..N、launch entry、result JSON、Boost/Gear契約を変更しない。

