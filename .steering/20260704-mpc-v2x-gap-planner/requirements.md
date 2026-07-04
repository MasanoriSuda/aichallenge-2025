# MPC V2X Gap Planner Requirements

作成日: 2026-07-04
状態: Draft

## 目的

現行 C++ MPC に、V2X で得られる他車位置を使った簡易的な追い越し・すり抜け判断を追加するための要求を整理する。

目的は、2台以上の他車が存在する場面で、事前 trajectory 追従だけに頼らず、通過可能な横方向 gap を選び、その corridor を MPC 制約または目標へ反映できる構造を作ることである。

## 現状認識

現行 C++ MPC には、コース境界に対する横方向制約はある。

- reference path の `lb/ub` を horizon ごとの横方向制約として使う。
- `safety_margin_scale` が 0 より大きい場合は、車幅由来の safety margin で境界から離す。
- `safety_margin_scale: 0.0` の場合は、車両中心点に近い境界制約となり、車体接触の保証は弱くなる。

一方で、他車を避ける構造は現状では有効に機能していない。

- `aichallenge_submit_launch/launch/control/mpc.launch.xml` の `use_obstacle_avoidance` は既定 `false`。
- `multi_purpose_mpc_ros/config/config.yaml` の `use_path_constraints_topic` と `use_border_cells_topic` は既定 `false`。
- C++ 側は `use_obstacle_avoidance=true` のとき `/v2x/vehicle_positions` を subscribe する入口を持つ。
- ただし C++ 側 callback は空で、受信した他車位置を障害物・制約・目標に変換していない。
- Python 側には V2X tracker、obstacle manager、path constraints provider の材料があるが、現在の C++ MPC 通常起動には接続されていない。

したがって現状の通常起動は、他車の隙間を見てすり抜けるのではなく、事前 trajectory とコース境界制約に従って走るだけである。

## 対象範囲

対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- `aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/mpc.launch.xml`
- 必要に応じて `multi_purpose_mpc_ros` 内の Python 実装を参照する。
- 必要に応じて `docs/spec/mpc-integration.md` と `README.md` を更新する。

対象外:

- `/v2x/vehicle_positions` の topic 名・message 型変更。
- `/control/command/control_cmd` の topic 名・message 型変更。
- 評価 FSM、result JSON、AWSIM 管理、Domain 設計の変更。
- 公式 2026 仕様として未確認の動的障害物ルールを確定扱いすること。
- 高度な行動計画器、学習ベース planner、全局 trajectory 再生成。
- 実車・遠隔環境での未検証な追い越し実行。

## 必要機能

### V2X 入力処理

- `/v2x/vehicle_positions` を受け取る。
- 自車を除外する。
- 他車の現在位置、姿勢、速度が取得できる場合は保持する。
- 取得できない値がある場合は conservative な既定値で扱う。
- 古い V2X 情報は timeout で破棄する。

### 他車予測

- 初期実装は等速直線予測でよい。
- horizon 時刻ごとに他車位置を予測する。
- 予測半径または矩形 footprint を config 化する。
- 予測不確かさを margin として膨張させる。

### Frenet / path 座標変換

- 他車予測位置を reference path 上の `s` と横方向 offset に変換する。
- MPC horizon の各 waypoint と、近い `s` 範囲にいる他車を対応付ける。
- reference path の `lb/ub` を base corridor として使う。

### Gap 生成

- 各 horizon 点で、コース境界 `lb/ub` から他車占有区間を差し引く。
- 残った横方向 interval を gap 候補として扱う。
- gap 幅が `vehicle_width + margin` 未満なら候補から除外する。
- 2台の間に十分な幅がある場合は中央 gap を候補にする。
- 左右外側に十分な幅がある場合は外側 gap も候補にする。

### Gap 選択

初期実装は rule-based とする。

優先度の例:

1. 現在の横位置から急に離れすぎない gap。
2. reference trajectory から離れすぎない gap。
3. 幅が広い gap。
4. horizon 全体で継続して存在する gap。
5. 操舵変化が小さい gap。

通過可能 gap がない場合は、無理に突っ込まず減速または停止へ倒す。

### MPC 反映

選択した gap を次のいずれかで MPC に渡す。

- `lb/ub` を gap corridor に狭める。
- `xr` の横方向目標を gap 中央へ寄せる。
- 両方を行う。

初期実装では、既存の path constraint 構造に合わせて `lb/ub` を更新し、必要に応じて `xr` を gap 中央へ寄せる方針が自然である。

## Config 要求

少なくとも次を config 化する。

```yaml
mpc:
  use_v2x_gap_planner: false
  v2x_vehicle_radius: 1.25
  v2x_prediction_margin: 0.2
  v2x_prediction_time: 3.0
  v2x_timeout_sec: 1.0
  gap_min_width: 1.8
  gap_target_bias: 1.0
  no_gap_target_velocity: 0.0
```

既定値では現行挙動を維持するため、`use_v2x_gap_planner` は `false` とする。

## 安全要求

- planner が不確実なときは、すり抜けではなく減速または停止へ倒す。
- V2X 情報が stale の場合は使わない。
- gap 幅が足りない場合は候補にしない。
- horizon 途中で gap が消える場合は候補にしない、または速度を落とす。
- `safety_margin_scale=0.0` と組み合わせる場合は接触リスクを明記する。
- 実車では使用しない。シミュレータで `make dev` または `make gate*` の確認後に扱う。

## 互換性要求

- 既定設定では現在の C++ MPC 挙動を変えない。
- `control_method=mpc` の launch 経路を維持する。
- `/v2x/vehicle_positions`、`/control/command/control_cmd`、`/localization/kinematic_state`、`/planning/scenario_planning/trajectory` の topic 契約を変えない。
- `aichallenge_system/` 側を変更しない。
- config key が未指定でも起動できる。

## 受け入れ条件

- `use_v2x_gap_planner=false` で現行挙動を維持する。
- `/v2x/vehicle_positions` を受信して他車リストを更新できる。
- 他車が前方にいない場合は通常 trajectory 追従になる。
- 前方に1台いる場合、左右に十分な幅があれば gap corridor を生成できる。
- 前方に2台いる場合、2台の間に十分な幅があれば中央 gap を候補にできる。
- 通過可能 gap がない場合は減速または停止目標を出せる。
- `make autoware-build` が成功する。
- シミュレータ上で `/control/command/control_cmd` の publish を維持する。

## 未確定事項

- `/v2x/vehicle_positions` の field から速度・yaw をどこまで信頼できるか。
- 他車 footprint を円で近似するか、矩形で扱うか。
- gap corridor を `lb/ub` に反映するか、`xr` のみに反映するか、両方にするか。
- no-gap 時に速度目標を MPC 内で下げるか、別の停止制御へ渡すか。
- 評価シナリオで追い越し・すり抜けが加点になるのか、ペナルティリスクが高いのか。
