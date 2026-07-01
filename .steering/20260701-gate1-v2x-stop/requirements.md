# Gate1 V2X Stop Requirements

## Background

Automotive AI Challenge 2026 の Gate1 は障害物停止を対象にする。2026-07-01 の運営回答により、障害物情報の入力は `/v2x/vehicle_positions` のみを使用する。

既存の LiDAR、Camera、CSV 障害物、`/aichallenge/objects` は Gate1 の公式入力として使わない。

## Goal

MPC 制御で走行中、V2X 上の前方停止対象に対して、追い越しせずに安全に減速・停止する。

Gate1 の最小ゴール:

1. `/v2x/vehicle_positions` から前方停止対象を検出する。
2. 自車の参照経路上で前方にいる対象だけを停止対象にする。
3. 距離に応じて速度上限を下げる。
4. 停止必要距離以下では停止し、停止後に不要な再加速をしない。
5. 停止判断の根拠を log / rosbag から追える。

## Scope

変更してよい主対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- `aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/mpc.launch.xml`
- `docs/spec/` と `docs/interface/` の追記

変更しない対象:

- `aichallenge/workspace/src/aichallenge_system/`
- `/v2x/vehicle_positions` の topic 名、message 型、fanout 設計
- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- result JSON schema

## Functional Requirements

### R1. V2X input

- Gate1 停止判断は `/v2x/vehicle_positions` のみを入力にする。
- message は現行ローカルの `v2x_msgs/V2XVehiclePositionArray` を前提にする。
- `vehicle_id`、`position`、`header.stamp` を使用する。
- 相対速度は既存 `V2XVehicleTracker` の有限差分推定を使う。

### R2. Candidate filtering

- 自車近傍の V2X 点は self echo とみなして除外する。
- 自車後方の対象は停止対象にしない。
- 参照経路の進行方向上で前方にあり、かつ横方向に近い対象だけを停止対象にする。
- 複数対象がある場合は、進行方向距離が最短の対象を採用する。
- 座標ジャンプ、異常速度、NaN / Inf を含む対象は無視し、警告を出す。

### R3. Speed cap

- 通常の `ref_vel.yaml` / `v_max` で決まる速度上限に、V2X 停止用の速度 cap を重ねる。
- `gap` が十分大きい場合は通常走行を妨げない。
- `gap` が小さくなるほど速度 cap を下げる。
- 停止目標距離 `target_stop_gap_m` 以下では速度 cap を `0.0` にする。
- 停止中は hysteresis を持たせ、対象が少し揺れても再加速しない。

### R4. Braking behavior

- 減速計算は `mpc.a_min` より強い減速を要求しない。
- 速度 cap は以下の形を基本にする。

```text
available_gap = max(gap - target_stop_gap_m, 0)
v_cap = sqrt(2 * comfortable_decel_mps2 * available_gap)
```

- `comfortable_decel_mps2` は `abs(mpc.a_min)` 以下の設定値にする。
- 近距離では停止優先とし、ラップタイム最適化より接触回避を優先する。

### R5. Fail-safe

- V2X が未受信の場合は通常走行に戻す。ただし停止保持中に短時間欠損した場合は停止を維持する。
- V2X が stale timeout を超えた場合は、対象なしとして扱い、警告を出す。
- 入力異常で停止対象が不確かな場合は、近距離側では停止、安全距離外では通常走行へ倒す。

### R6. Observability

- 最低限、以下を throttled log で出す。
  - active / clear / holding stop
  - selected `vehicle_id`
  - `gap`
  - `lateral_offset`
  - `v_cap`
- Gate1 解析では以下 topic を rosbag / log で確認できること。
  - `/v2x/vehicle_positions`
  - `/localization/kinematic_state`
  - `/planning/scenario_planning/trajectory`
  - `/control/command/control_cmd`

## Non-Goals

- Gate1 では追い越し判断を実装しない。
- Gate1 では横方向の回避ライン生成を実装しない。
- `path_constraints_provider` による高度な上下限制約は必須にしない。
- LiDAR / Camera ベースの障害物認識は扱わない。

## Definition Of Done

- `make autoware-build` が通る。
- 追加した pure Python ロジックに単体テストがある。
- `make gate1` で `/v2x/vehicle_positions` 由来の前方対象に対して減速し、接触せず停止する。
- 停止後に不要な再加速をしない。
- `output/latest/d<N>/autoware.log` から停止判断の根拠を追える。
- 仕様変更があれば `docs/spec/safety-gates.md` または `docs/spec/mpc-integration.md` に反映されている。
