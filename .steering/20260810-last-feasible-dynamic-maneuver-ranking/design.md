# Design

## 方針

既存の `PassManeuverCandidateAssessment` と
`OpponentSideManeuverComparison` を候補生成・rankingの正本として再利用する。
新しいplannerは増やさず、評価済みMissionの寿命とsoft failureへの接続だけを追加する。

## 状態

`OvertakeLineState`へ次を保持する。

- last feasible current-side Mission / evaluated time
- last feasible alternate-side Mission / evaluated time
- alternateがdebounceを完了していたか
- cache対象target IDとMission generation

Missionの新規freeze、target変更、Idle resetでcacheを破棄する。評価で一時的に候補が
得られなくても即座には消さず、有効期限で失効させる。

## 再利用policy

pure functionで以下を判定する。

1. hard fault、target discontinuity、実車体重複ならblocked。
2. target ID / generation不一致またはcache期限超過ならstale/unavailable。
3. no-return前かつ安定済みalternateがあればalternateを選択。
4. それ以外でfresh current-side候補があればcurrentを選択。
5. なければ既存fallbackへ移る。

SafeSeparationのlocal time/distanceとshort-horizonのsoft abortでpolicyを呼ぶ。
short-horizon不成立時は、同じ経路を再利用せず、no-return前の安定済みalternateだけを
対象にする。absolute Pass limit、absolute Mission total budget、物理hard faultは
従来どおりterminalとする。

## 原子的差し替え

既存の `replace_frozen_overtake_mission_after_dynamic_replan()` を使用する。
Pass累積時間・距離・Mission総時間を保存したまま、PassPlan全体を一度に置換する。
同側置換時は旧generationを無効化した上でfresh generationへ更新する。

## 設定

- `v2x_overtake_last_feasible_maneuver_enabled`
- `v2x_overtake_last_feasible_maneuver_max_age_sec`

既存の評価周期0.15秒、安定時間0.25秒、no-return距離2.0mをそのまま利用する。
最大side replacement回数は、動的再評価を実走で確認できるよう1回から3回へ増やす。

## ログ

- cache rescue accepted: current/alternate、age、abort reason
- cache rescue rejected: policy reason（debug有効時のイベントのみ）

毎周期ログは追加しない。
