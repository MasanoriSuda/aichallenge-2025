# Design

## 背景

`20260816-091913` では execution envelope 有効化後、rolling candidate pending 12件のうち11件が measured-state rebase を使用していた。静的壁の実寸制約違反は0件だった一方、11件が `ay=1`、全件が `max_ay=6.00` であり、再ベース候補の昇格は1件に留まった。

現行DPの到達可能区間は実行validatorと同じ最大横加速度を境界に使う。DPがその境界を選ぶと、path補間・stitch・離散化の微小差で後段validatorが横加速度補正を行う。rolling refreshは補正済みhorizonを昇格させないため、fresh解がpendingとなりlast-feasible pathを長く保持する。

## 方針

### 計画用横加速度余裕

execution envelope requestへ `lateral_accel_reserve_ratio` を追加し、到達可能区間を次で計算する。

```text
planning_max_ay = execution_max_ay * reserve_ratio
reachable(t) = current_ey + current_lateral_velocity * t
               +/- 0.5 * planning_max_ay * t^2
```

- 既定値は1.0とし、機能未設定時の互換性を維持する。
- 有効範囲は `(0, 1]` とする。
- 競技configでは0.90とし、6.0 m/s²の実行上限に対して5.4 m/s²をDP計画上限とする。
- 実行validatorは従来どおり6.0 m/s²を使用する。

これにより安全判定を緩和せず、DP解に補間・stitch・追従誤差のための10%余裕を残す。

### 局所リファクタ

比率適用をcontroller側の場当たり的な乗算にせず、pure policy内で入力検証・実効値計算・到達区間生成を一体化する。resolutionへ実効上限を返し、単体テスト可能にする。

## 設定

```yaml
v2x_overtake_mpcc_frenet_dp_execution_lateral_accel_reserve_ratio: 0.90
```

ローカル・提出configの双方へ同値を設定する。

## ログ

- 起動ログ: `ay_reserve` を表示する。
- rate-limited pendingログ: 後段の `max_ay` とDPの `plan_ay` を併記する。

周期ログは追加しない。

## 互換性

参加者ROS I/O、評価FSM、Domain、launch entry、提出物構造、result JSONを変更しない。
