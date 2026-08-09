# Requirements

## 背景

`output/20260809-005637/d1/autoware.log` では `ShiftOut -> Pass` が8回成立した一方、
`Pass -> Return` は0回だった。候補選択時は加速度・制御遅延・コース速度上限を含む
kinematic rolloutを使うが、Pass実行中のforward-completion latchは一定closing速度の
割り算へ戻っており、予測契約が一致していない。また、横経路は基本的に固定
`goal_ey`で、状況変化は大きなMission置換または救済FSMで処理している。

## 目的

1. Pass実行時のrear-clear予測を候補選択と同じkinematic rolloutへ統一する。
2. no-return後もsideは固定し、同一side内の安全な横goalを短周期・小変位で更新する。
3. Pass、FollowPrepare、代替Missionを合算した同一targetの総処理時間を制限する。
4. 既存の壁、footprint、横加速度、target continuity、ContactContinuationのhard guardを維持する。

## 変更範囲

- `multi_purpose_mpc_ros` のV2X overtake core、controller、設定、単体テスト。
- 参加者実装内に閉じ、ROS topic/service/message契約と評価基盤は変更しない。

## 非目標

- MPPI/MPCCへの全面置換。
- 毎周期の左右side反転。
- 接触を意図的に探索する制御。
- AWSIMまたは評価システム側の変更。

## Definition of Done

- 実行時forward-completionが加速度制約付きrolloutの結果だけで新規latchされる。
- 動的corridor更新は同一side、小変位、cooldown、full-path preflight、rolloutを通過した場合だけcommitされる。
- 同一targetの総Mission時間が設定値を超えた場合、rear-clearならReturn、それ以外はboundedな終了処理へ移る。
- core単体テストと対象package buildが成功する。

