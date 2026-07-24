# 5 m追従単独分離実験 要件

## 目的

先行車d2を16 km/hに制限したdev2で、追い越し横経路とRecoveryを一時的に抑止し、
d1が5 m付近で急減速せず安定追従できるか確認する。

追い越し失敗によるRecovery減速と、Follow速度制限そのものによる減速を分離する。

## ベースライン

- run: `output/20260724-235653`
- d2 `domain_v_max`: 16.0 km/h
- d1 Lap 1 / 2: 75.193 / 74.948 s
- d2 Lap 1 / 2: 73.694 / 75.023 s
- d1 Recovery: bag全体57回
- d1/d2とも5トピックMCAPを取得済み

## 実験条件

- dev2、d1/d2、2周以上
- d2の16 km/h制限、Follow距離・速度設定、安全制約は変更しない
- Follow FSMとV2X追跡は有効なまま維持する
- 実験中だけ次を変更してOvertake成立を抑止する
  - `v2x_start_grid_breakout_enabled: false`
  - `v2x_overtake_min_gap_width: 100.0`
- 測定後は上記2項目を元の値へ戻す

## 変更禁止範囲

- C++ / Pythonソースロジック
- Follow距離・速度・加速度パラメータ
- wall margin、EmergencyBrake、Stuck Recovery
- ROS 2 topic/service/message、Domain、評価schema
- `output/`とMCAPの編集

## 観測項目

1. Overtake / OvertakeLine / Recoveryが0回であること
2. Follow中の`fd`、`follow_cap`、速度上限
3. 5 m付近のcommand speed、実速度、実加速度
4. d1-d2車間の平均、最小、ばらつき
5. 急な速度指令低下と負加速度の発生回数
6. SafetyStop、runtime contact、active Reverseの有無
7. lap timeとMCAPのtopic健全性

## 判定

- 追従単独でも急失速する:
  - 5 m Follow境界または移動先行車速度制限を優先修正する。
- 追従単独では安定する:
  - 急失速の主因はOvertakeLine Recoveryであり、追い越し開始・再試行条件を優先する。
- Overtake / Recoveryが残る:
  - 分離実験として無効なので、結果判定に使わない。

