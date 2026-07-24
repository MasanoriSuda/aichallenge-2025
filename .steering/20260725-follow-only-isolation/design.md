# 5 m追従単独分離実験 設計

## 分離方法

`use_v2x_behavior_fsm`を無効化するとFollow制御自体も止まるため使用しない。
`v2x_overtake_line_enabled`だけを無効化するとlegacy横目標がOvertakeを引き継ぐため、
これも追従単独条件にならない。

そこでV2X Behavior FSMとgap plannerは動かしたまま、追い越し候補だけを成立不能にする。

```yaml
v2x_start_grid_breakout_enabled: false
v2x_overtake_min_gap_width: 100.0
```

- start grid専用例外を無効化し、走行開始直後のOvertakeを防ぐ。
- 通常のpass side候補は100 mの通路幅を満たせないため、BehaviorはFollowへ残る。
- `v2x_follow_gap_planner_enabled: false`は現行値のままなので、Followはベース経路を使う。
- d2は約4.44 m/sであり、停止車向けLowSpeedAvoidanceの対象外である。

## A/B比較

| 条件 | run | Overtake |
|---|---|---|
| A | `20260724-235653` | 現行有効 |
| B | 今回取得 | 一時抑止 |

両方ともd2 16 km/h、Recovery上限5.0 m/s、同じ5トピックMCAPを使用する。

## 時刻同期

AutowareログのV2X debugから`final=Follow`、`fd`、`follow_cap`を取得し、その時刻へ
MCAPの次の値を合わせる。

- `/control/command/control_cmd`: 指令速度・指令加速度
- `/localization/kinematic_state`: 実速度・自車位置
- `/localization/acceleration`: 実加速度
- `/v2x/vehicle_positions`: d2位置
- `/clock`: 記録時刻連続性

V2X車間はd1 odometryとd2 V2X位置のmap座標距離も算出する。ただしヘアピンでは
ユークリッド距離が沿道距離と一致しないため、5 m境界判定の正本はdebugの`fd`とする。

## 急減速指標

- 5 m境界の前後2秒におけるcommand / actual speedの最大低下
- 同区間の最小実加速度
- 5 m付近での`follow_cap`出入り回数
- 追従区間でd1実速度がd2より0.5 m/s以上低い時間

同じ計算をA/Bへ適用し、Overtake除去後にも残る急減速だけをFollow固有事象とする。

## 後処理

本実験の設定は診断用であり、測定終了後に次へ戻す。

```yaml
v2x_start_grid_breakout_enabled: true
v2x_overtake_min_gap_width: 0.2
```

採用するFollow調整値は、この実験結果を根拠に別ステアリングで一項目ずつ決める。

