# Requirements

## 背景

最新走行では Pass 進入後の longitudinal horizon refresh 自体は動作したが、
継続 preflight が初回進入と同じ条件を再適用したため、主に次の理由で
SafeSeparation / Recovery へ移行している。

- `outer pass becomes inside before rear-clear`
- `target separation does not fit wall-feasible bounds`

狭いヘアピン主体のコースでは、固定した同一横経路を走り続けても曲率符号により
「外側」が「内側」へ変わる。また、車体 footprint が非重複でも中心間 1.5 m を
満たさない場合がある。これらを Pass 継続時まで hard reject にすると、既に横へ
出た車両を不要に減速・Recovery させる。

## 目的

- 初回の ShiftOut / Pass 経路採用条件は維持する。
- 固定横目標を変更しない longitudinal refresh に限り、車体 footprint による
  現在・予測非重複を中心とした継続判定へ切り替える。
- 壁、横加速度、操舵曲率、target continuity、EmergencyBrake の保護は維持する。
- `a_max: 1.0` を含む速度・加速度設定は変更しない。

## 変更範囲

- `v2x_overtake_core`: Pass 継続 preflight policy の純粋判定と単体テスト
- `mpc_controller_cpp`: preflight の初回採用と longitudinal continuation の分離
- 本 steering 配下の設計・検証記録

## 非対象

- 初回追い越し候補の gap / wall margin 緩和
- geometric same-side extension の安全条件変更
- Recovery / reverse 制御の変更
- ROS 2 topic、message、launch、評価 schema の変更

## Definition of Done

- longitudinal refresh でのみ footprint continuation policy が選択される。
- current footprint、predicted footprint sweep、target continuity のいずれかが
  不成立なら従来の hard constraint を緩和しない。
- policy 有効時も rear-clear rollout と full-path の壁・横加速度・操舵曲率検証を通す。
- 初回進入と geometric extension は中心間離隔・outer-role 条件を維持する。
- core 単体テストと `make autoware-build` が成功する。
