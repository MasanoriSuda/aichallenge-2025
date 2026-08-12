# Requirements

## 背景

`output/20260812-135806/d1/autoware.log` では、自車が約 9.81 m/s、停止車まで
9.23 m の時点で `LowSpeedAvoidance` の直接操舵へ移行した。直接操舵は 3.0 m/s
を要求し、カーブ中の直前操舵を中心とする制約を引き継いだ後、約 2 秒で壁余裕
違反、`e_y=-1.835 m`、Stuck Recovery へ至った。

## 目的

- 直接操舵の目標速度まで物理的に減速できない高速状態では、LowSpeedAvoidance
  のローカル経路を MPC に追従させる。
- 低速で直接操舵を使う場合も、直前操舵だけではなく現在のコース曲率を基準に
  回避操舵補正を制限する。
- 停止車両の検出とローカル経路生成自体は維持する。

## 制約

- `/control/command/control_cmd` などの ROS 2 契約を変更しない。
- 壁、車体重複、Emergency、solver failure の既存guardを緩和しない。
- 高速時にLowSpeedAvoidance全体を無効化せず、MPCによる回避経路追従を残す。
- `aichallenge/result-summary.json` の既存変更には触れない。

## Definition of Done

- 20260812-135806相当の条件でdirect-control入口が拒否される単体テストがある。
- 減速距離が足りる低速条件ではdirect-control入口を維持する。
- direct操舵範囲が基準曲率、直前操舵レート、物理操舵上限を考慮する。
- `multi_purpose_mpc_ros` のビルドと単体テストが成功する。
