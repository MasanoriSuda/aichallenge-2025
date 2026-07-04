# MPC Wall-Edge Trajectory Tracking Config Design

作成日: 2026-07-04
状態: Draft

## 方針

既存 C++ MPC の外部インターフェースは変えず、MPC 内部の横方向目標と safety margin を config 化する。

デフォルト値は現行挙動を維持する。壁寄せ検証時だけ `config.yaml` で値を下げる。

## 現行ロジック

### 中央寄せ目標

`MPC::init_problem()` で horizon 上の左右制約 `lb/ub` を取得し、横方向の reference `xr` を中央に置いている。

```cpp
xmin_dyn[nx + i * nx] = lb[i];
xmax_dyn[nx + i * nx] = ub[i];
xr[nx + i * nx] = (lb[i] + ub[i]) / 2.0;
```

ここで `e_y=0` は CSV trajectory 線上を意味する。現行実装は `e_y=0` ではなく、左右制約中央を目標にしている。

### safety margin

`BicycleModel` 初期化時に `compute_safety_margin()` で margin を計算する。

```cpp
safety_margin = width / std::sqrt(2.0);
```

`ReferencePath::update_simple_path_constraints()` で左右制約に margin を反映する。

```cpp
double ub_sm = wp.ub - safety_margin;
double lb_sm = wp.lb + safety_margin;
```

## 設計案

### Config structure

`MpcConfig` に次を追加する。

```cpp
double center_bias{1.0};
double safety_margin_scale{1.0};
```

`load_config()` で `mpc.center_bias` と `mpc.safety_margin_scale` を読む。

互換性のため、YAML key が存在しない場合は次の既定値を使う。

```cpp
center_bias = 1.0;
safety_margin_scale = 1.0;
```

### center bias

`MPC::init_problem()` の横方向 reference を次の形にする。

```cpp
const double center_ey = (lb[i] + ub[i]) / 2.0;
xr[nx + i * nx] = cfg.center_bias * center_ey;
```

意味:

- `center_bias=1.0`: 現行挙動。
- `center_bias=0.0`: trajectory 線を目標にする。
- `center_bias=0.5`: trajectory と中央の中間を目標にする。

範囲外値の扱い:

- 初期実装では `0.0 <= center_bias <= 1.0` に clamp する。
- clamp した場合は warning log を出すか、少なくとも config コメントで範囲を明記する。

### safety margin scale

`BicycleModel` に scale を渡せるようにする。

候補:

```cpp
BicycleModel(
  ReferencePath * ref_path,
  double length_in,
  double width_in,
  double safety_margin_scale_in,
  double Ts_in)
```

計算:

```cpp
double compute_safety_margin() const
{
  return width / std::sqrt(2.0) * safety_margin_scale;
}
```

範囲外値の扱い:

- `safety_margin_scale < 0.0` は `0.0` に clamp。
- `safety_margin_scale > 1.0` は許可してもよいが、初期運用では `1.0` までを推奨する。

### YAML example

互換性重視の既定:

```yaml
mpc:
  center_bias: 1.0
  safety_margin_scale: 1.0
```

壁寄せ検証の初期値:

```yaml
mpc:
  center_bias: 0.0
  safety_margin_scale: 1.0
```

次段階:

```yaml
mpc:
  center_bias: 0.0
  safety_margin_scale: 0.7
```

## 変更対象

### `mpc_controller_cpp.cpp`

- `MpcConfig` に field 追加。
- `load_config()` に YAML 読み込み追加。
- `BicycleModel` constructor に `safety_margin_scale` を渡す。
- `compute_safety_margin()` を scale 対応にする。
- `create_map_ref_path_car_mpc()` と `create_reference_path_from_autoware_trajectory()` の `BicycleModel` 初期化経路を確認する。
- `MPC::init_problem()` の `xr` 計算を `center_bias` 対応にする。

### `config.yaml`

- `mpc.center_bias`
- `mpc.safety_margin_scale`
- コメントで推奨テスト順を簡潔に記載する。

### `sim_config.yaml`

- 通常走行と同じ構造を保つなら同 key を追加する。
- `mpc_simulation.launch.py` を使う検証が不要なら後回しでもよい。

### Docs

実装時に設定値の意味を `README.md` または `docs/spec/mpc-integration.md` に追記する。

## 検証設計

### Static / build

- `make autoware-build`
- YAML key 欠落時に fallback が効くことを確認する。

### Runtime

最低限:

- `control_method=mpc` で node 起動。
- `/control/command/control_cmd` publish 確認。

推奨:

- `center_bias=1.0`, `safety_margin_scale=1.0` の baseline run。
- `center_bias=0.0`, `safety_margin_scale=1.0` の run。
- `center_bias=0.0`, `safety_margin_scale=0.7` の run。
- wall / over / crash penalty、lap time、制御出力、trajectory との横偏差を比較する。

## リスク

- `center_bias=0.0` は CSV trajectory の品質に強く依存する。
- `safety_margin_scale` を下げると localization delay、steering delay、速度上昇時の横滑りで壁接触しやすくなる。
- `Q[0]` が大きい設定では trajectory 追従が強くなり、制約近傍で infeasible fallback が増える可能性がある。
- `safety_margin_scale=0.0` は評価用というより限界確認用として扱う。

## ロールバック

- `center_bias: 1.0`
- `safety_margin_scale: 1.0`

この2値に戻せば現行の中央寄せ + 現行 margin に戻る設計にする。
