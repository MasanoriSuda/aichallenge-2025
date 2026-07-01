# Gate1 V2X Stop Design

## Summary

Gate1 は横回避ではなく、V2X で見える前方対象に対する縦方向の速度制限として実装する。

既存の `use_obstacle_avoidance` は MPC の障害物制約計算に使われるため、Gate1 停止ロジックとは分ける。Gate1 用には `use_v2x_stop` を追加し、`/v2x/vehicle_positions` の購読と速度 cap 適用を独立させる。

## Components

### `v2x_stop_planner.py`

新規 pure Python helper を追加する。

責務:

- `V2XVehiclePositionArray` 相当のデータを受ける。
- 自車 pose、参照経路、現在速度から前方対象を選ぶ。
- V2X 停止用の速度 cap と状態を返す。
- ROS 依存を持たせず、単体テストしやすくする。

想定 API:

```python
planner = V2XStopPlanner(config)
planner.update_v2x(msg)
result = planner.compute_speed_cap(
    ego_x=pose.x,
    ego_y=pose.y,
    ego_yaw=pose.theta,
    ego_v_mps=v,
    reference_xy=waypoint_xy,
    now_sec=now_sec,
)
```

`result` の想定:

```python
V2XStopResult(
    active=True,
    holding_stop=False,
    speed_cap_mps=2.56,
    reason="braking",
    vehicle_id="d2",
    gap_m=6.4,
    lateral_offset_m=0.8,
    relative_speed_mps=-1.0,
)
```

### `mpc_controller.py`

変更方針:

- `use_v2x_stop` parameter を追加する。
- `use_v2x_stop=true` のとき `/v2x/vehicle_positions` を購読する。
- 既存 `use_obstacle_avoidance` が false でも V2X 停止は動くようにする。
- 既存 `V2XVehicleTracker` は相対速度推定に使う。
- `_ref_vel_configulator` の速度上限を計算した後、V2X 停止 cap と `min()` する。

速度上限の適用位置:

```text
base_v_max = ref_vel_configulator or mpc.v_max
v2x_cap = v2x_stop_planner.compute_speed_cap(...)
effective_v_max = min(base_v_max, v2x_cap)
_mpc.update_v_max(effective_v_max)
_reference_path.set_v_ref([effective_v_max] * len(waypoints))
```

注意:

- 単位は明示的に扱う。既存 `mpc.v_max` は km/h コメントだが、コード上の扱いを実測してから揃える。
- `kmh_to_m_per_sec` の既存利用は命名と実処理を再確認する。
- 停止 cap は制御周期ごとに滑らかに変える。急な 30 -> 0 の段差は避ける。

### Config

`config.yaml` と `sim_config.yaml` に `v2x_stop` section を追加する。

初期値案:

```yaml
v2x_stop:
  enabled: true
  detection_range_m: 25.0
  corridor_half_width_m: 1.8
  self_ignore_radius_m: 0.75
  target_stop_gap_m: 3.0
  stop_hold_gap_m: 3.5
  release_gap_m: 5.0
  comfortable_decel_mps2: 1.2
  stale_timeout_sec: 2.0
  max_speed_cap_kmph: 30.0
  log_throttle_sec: 1.0
```

`comfortable_decel_mps2` は `abs(mpc.a_min)` を超えないように clamp する。

### Launch

`aichallenge_submit_launch/launch/control/mpc.launch.xml` に以下を追加する。

```xml
<arg name="use_v2x_stop" default="true"/>
<param name="use_v2x_stop" value="$(var use_v2x_stop)"/>
```

Gate1 だけでなく通常 race でも安全停止が効くよう、既定は true とする。V2X 未受信なら通常走行に戻るため、単独 dev の挙動は大きく変えない。

## Candidate Selection

1. 自車 pose を `/localization/kinematic_state` から取得する。
2. 参照経路 `reference_xy` から自車最近傍 index を求める。
3. 各 V2X 対象について、対象最近傍 index を求める。
4. circular path として前方距離 `gap_m` を求める。
5. 対象の path からの横方向距離 `lateral_offset_m` を求める。
6. 以下を満たす対象だけ残す。
   - `0 < gap_m < detection_range_m`
   - `lateral_offset_m < corridor_half_width_m`
   - ego からの直線距離が `self_ignore_radius_m` より大きい
7. `gap_m` が最小の対象を採用する。

参照経路が一時的に使えない場合は、自車 yaw の前方 dot product で fallback する。

## Speed Policy

状態は 3 つに分ける。

| State | 条件 | 出力 |
|---|---|---|
| clear | 前方対象なし | 通常速度 |
| braking | 前方対象あり、停止距離外 | `sqrt(2*a*gap)` ベースの速度 cap |
| holding_stop | `target_stop_gap_m` 以下または停止済み | `0.0` |

hysteresis:

- `holding_stop` から clear へ戻すには `release_gap_m` 以上の余裕が必要。
- V2X が短時間欠損しても `stale_timeout_sec` までは最後の停止対象を保持する。

## Validation

単体テスト:

- 前方対象だけ選ばれる。
- 後方対象は無視される。
- 横方向に遠い対象は無視される。
- self echo は無視される。
- gap が縮むと speed cap が下がる。
- target stop gap 以下で speed cap が 0 になる。
- stale timeout と hold/release hysteresis が効く。

実機能確認:

```bash
make autoware-build
make gate1
```

観測:

```bash
ros2 topic echo --once /v2x/vehicle_positions
ros2 topic hz /control/command/control_cmd
ros2 topic echo --once /control/command/control_cmd
```

確認ログ:

- `output/latest/d<N>/autoware.log`
- `output/latest/d<N>/result-summary.json`
- `output/latest/d<N>/result-details.json`
- `output/latest/d<N>/rosbag2_autoware.mcap`

## Risks

- `/v2x/vehicle_positions` に自車が含まれる場合、自車を障害物として誤検出する。
- 座標系が map frame でない場合、参照経路との比較が破綻する。
- `v_max` の単位がコード上で混在している可能性がある。
- 停止距離を攻めすぎると Gate1 で接触する。
- 停止保持が強すぎると、Gate2 / race behavior で追い越し判断へ移れない。

## Migration To Gate2

Gate2 では、この Gate1 の前方対象検出をそのまま使う。違いは、`braking` だけでなく、追い越し可能な場合に lateral behavior へ分岐する点。

Gate1 では「前方対象あり、追い越し不可なら止まる」を完成させる。
