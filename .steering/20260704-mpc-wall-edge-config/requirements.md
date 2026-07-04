# MPC Wall-Edge Trajectory Tracking Config Requirements

作成日: 2026-07-04
状態: Draft

## 目的

C++ MPC が編集済み trajectory を壁寄せ方向に追従できるように、現在コード内に固定されている中央寄せ目標と safety margin を config から段階調整できるようにする。

目的は「壁際を走らせるための調整面を作ること」であり、MPC 全体の再設計や高速化、trajectory CSV の作り直しではない。

## 背景

現行 C++ MPC は、reference trajectory を基準に左右の走行可能幅を計算し、その中央を横方向目標にしている。

```cpp
xr[nx + i * nx] = (lb[i] + ub[i]) / 2.0;
```

このため trajectory editor で CSV を壁側に寄せても、MPC の最適化目標が左右制約の中央になり、走行軌跡がセンター側へ戻る。

また、path constraint は `BicycleModel::compute_safety_margin()` の値で左右制約を狭める。

```cpp
return width / std::sqrt(2.0);
```

`bicycle_model.width: 1.45` の場合、safety margin は約 1.03 m になる。trajectory が壁からこの範囲内に入ると、中央寄せを消しても制約で押し戻される。

## 対象範囲

対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- 必要に応じて `sim_config.yaml`
- 必要に応じて `docs/spec/mpc-integration.md`
- 必要に応じて `multi_purpose_mpc_ros/README.md`

対象外:

- trajectory CSV の直接編集。
- trajectory editor の挙動変更。
- `simple_trajectory_generator` の経路生成変更。
- `simple_pure_pursuit` の制御ロジック変更。
- OSQP 問題の高速化、solver workspace 再利用。
- `/control/command/control_cmd` など ROS topic 契約の変更。
- `aichallenge_system/`、評価 FSM、result JSON、Domain 設計の変更。

## 追加する調整項目

`mpc` YAML 配下に次を追加する。

```yaml
mpc:
  center_bias: 1.0
  safety_margin_scale: 1.0
```

### `center_bias`

横方向目標 `xr` を trajectory 線と走行可能幅中央の間で補間する。

- `1.0`: 現行挙動。左右制約の中央を目標にする。
- `0.0`: trajectory 線そのものを目標にする。
- `0.0 < value < 1.0`: trajectory 線から中央側へ一部寄せる。

実装意図:

```cpp
const double center_ey = (lb[i] + ub[i]) / 2.0;
xr[nx + i * nx] = center_bias * center_ey;
```

### `safety_margin_scale`

既存の safety margin を倍率で調整する。

- `1.0`: 現行挙動。
- `0.5`: 現行 margin の半分。
- `0.0`: safety margin なし。

実装意図:

```cpp
safety_margin = width / std::sqrt(2.0) * safety_margin_scale;
```

## 互換性要求

- YAML key が存在しない場合でも現行挙動を維持する。
  - `center_bias` の fallback は `1.0`。
  - `safety_margin_scale` の fallback は `1.0`。
- `control_method=mpc` の launch 経路を変えない。
- node name、topic 名、message 型、QoS を変えない。
- `/control/command/control_cmd` を最終制御出力として維持する。
- `config.yaml` を変更しても `pure_pursuit`、`tiny_lidar_net`、`pilot_net`、`joycon` に影響させない。
- 2026 公式仕様として未確認の内容を確定扱いしない。

## 安全要求

- `safety_margin_scale=0.0` は実験用として扱う。
- 実車・遠隔環境ではシミュレータ確認なしに margin を下げない。
- 最初の検証は `center_bias` だけを下げ、次に `safety_margin_scale` を段階的に下げる。
- 壁寄せ trajectory が制約外になる場合は、MPC が無理に追従するのではなく、制約で抑制されることを許容する。

## 推奨検証順

1. `center_bias: 0.0`, `safety_margin_scale: 1.0`
   - 中央寄せだけを消す。
2. `center_bias: 0.0`, `safety_margin_scale: 0.7`
   - safety margin を少し削る。
3. `center_bias: 0.0`, `safety_margin_scale: 0.5`
   - 壁寄せ効果と接触リスクを見る。
4. 必要時のみ `safety_margin_scale: 0.3`
5. 最後の実験としてのみ `safety_margin_scale: 0.0`

## 受け入れ条件

- `make autoware-build` が成功する。
- `control_method=mpc` で C++ MPC node が起動する。
- `/control/command/control_cmd` が publish される。
- `center_bias=1.0`、`safety_margin_scale=1.0` で現行挙動を維持できる。
- `center_bias=0.0` で横方向目標が trajectory 線になる。
- `safety_margin_scale` を変更したとき、path constraint の左右余白が倍率通り変わる。
- 実走または `make dev` / `make gate*` で、壁寄せ trajectory の戻り具合を比較できる。

## 未確定事項

- どの `safety_margin_scale` まで評価で接触リスクを許容できるか。
- `center_bias` を runtime parameter として動的更新可能にする必要があるか。
- visualization marker に target `xr` を表示する必要があるか。
- `sim_config.yaml` に同じ key を追加するか、通常走行の `config.yaml` のみに限定するか。
