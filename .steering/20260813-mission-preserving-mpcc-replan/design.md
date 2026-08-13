# Design

## 1. 速度とtarget予測を同じ時間軸で評価する

現行のpost-validationは複数の候補速度を試すが、targetの相対位置制約は最初の
`speed_for_time` で一度だけ生成される。そのため速度を下げても、横離隔を得るまでの
時間が増える効果がtarget overlap windowへ反映されない。

候補速度ごとに次を再計算する。

- horizon sampleへ到達する時刻
- その時刻のtarget lateral
- nominal ego速度から候補速度へ変えた分を補正したtarget longitudinal
- 車体が縦方向に重なるsampleと、そのsampleの物理target境界

最初の最適化は現行速度を使う。post-validationだけ複数速度で再構成し、成立する中で
最も高い速度を採用する。

## 2. softなtarget境界失敗ではMissionを先に再計画する

次をすべて満たすtarget境界失敗は、壁接触等と分離して扱う。

- ShiftOutまたはPass中
- target観測が連続
- 現在車体が非重複、またはrecoverable side contact
- 壁接触、壁計測欠損、EmergencyBrakeではない

Freshな同側candidate、次にlast-feasible candidateを試し、成立すれば同じPass Missionの
時間・距離budgetを保ったまま置換する。候補がなければDynamicMissionWaitへ移し、
即時Recoveryは最終手段とする。

## 3. SafeSeparation soft abortの順序を変更する

`LocalTimeLimit`、`LocalDistanceLimit`、`ShortHorizonUnsafe`では、Freshな同側candidateを
DynamicMissionWaitより先に試す。これにより、通れる経路が同じ周期ですでに得られている
のに一度FollowPrepareへ落ちる不連続を避ける。

## 安全境界

以下はMission保持の対象外とする。

- actual wall physical contact / wall margin violation / wall sample unavailable
- target discontinuity / position jump
- non-recoverable body overlap
- EmergencyBrake
- forbidden waypoint / solver recovery threshold

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: 候補速度に整合した相対target予測の純粋関数
- `mpc_controller_cpp.cpp`: validation constraint再構築とMission保持型fallback
- `test_v2x_overtake_core.cpp`: 速度補正の単体テスト
