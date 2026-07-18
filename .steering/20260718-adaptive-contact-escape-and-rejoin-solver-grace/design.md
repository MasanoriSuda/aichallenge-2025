# Adaptive Contact Escape and Rejoin Solver Grace Design

作成日: 2026-07-18
状態: Completed（dev3受け入れは部分成功）

## Adaptive contact escape

`recovery_footprint`に最大操舵角を等分するpure helperを追加する。Side / Mixedかつ未commitの
stepwise候補では、Straight、Left 0.05..0.25 rad、Right 0.05..0.25 radを評価する。
contact減少数が最大の候補を選び、同値では先に評価した小さい操舵角を優先する。

`FeasibilityResult`へ候補のsigned steering angleを保持し、controllerがprimitiveだけでなく
角度もlatchする。commit後のstatic再評価、V2X corridor、実commandは同じ角度を使う。

## Rejoin solver grace

Recovery専有中のDrive報告はsolver fallback中でも受理してLowSpeedRejoinへ遷移できるようにする。
LowSpeedRejoinでsolverが不健康な間は専用timerを開始し、HoldStopを返す。timeout前に復旧すれば
timerをclearして通常のrejoin safety、alignment確認へ戻る。timeout後は従来のSolverUnsafe
SafeStopとする。

初回dev3で8 step・1.820 mまで安全に後退して2.0 m escape直前に上限となったため、step上限は
10へ増やす。最大後退距離3.0 m、各step停止・再評価、速度・時間上限は変更しない。

## Recovery episode budget

dev3で一度`rejoin_complete`したD2が、後の独立した詰まりへ前episodeの`step=10/10`を
持ち越して即SafeStopする不具合が判明した。完了episodeの最終ログにはstep数を残し、cooldown後に
新しいSuspected / Confirmedを受理してRecovery episodeを開始する時点でstep / attemptを0へ戻す。
SafeStop中の上限を自動解除する変更ではない。

## 安全性

- occupancy contact policy、swept step、footprint marginは変更しない。
- 操舵候補を増やすだけで、contact悪化候補を許可しない。
- solver待機中はaccelerationを出さず、Drive gearで停止する。
- V2X completeness、rear corridor、Boost inactive、GearReport gateは維持する。
- 実験値は2025 AWSIM向け暫定であり、実車・2026公式値ではない。

## 非対象

- footprint marginやoccupancy thresholdの緩和。
- Forward fallbackの再有効化。
- 複合的な前後切り返し。
- V2X messageへのRecovery状態追加。
