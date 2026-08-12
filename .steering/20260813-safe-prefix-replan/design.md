# Design

## 1. Safe trajectory prefix lease

SafeSeparation 中でも次をすべて満たす場合を `safe trajectory prefix` とする。

- Pass は tactical no-return 以降
- locked target の観測・進行が連続
- 現在車体が非重複
- footprint prediction が有効かつ sweep が非重複
- execution corridor が非閉塞
- 現在 Mission の静的検証済み prefix が設定距離以上残る
- 実壁、EmergencyBrake、solver の hard fault がない
- target 方向へ新鮮な前進進捗がある
- target が設定した前方範囲内

この lease は新しい安全性を仮定せず、既存の runtime hard guard と SafeSeparation absolute budget の内側だけで使用する。rear-clear rollout が一時的に不成立でも、既存の progress extension を利用して同じ側・速度を保持する。

## 2. Prefix 中の Mission 再探索

prefix lease 中は既存の左右候補評価を継続する。局所 SafeSeparation 上限へ近づいた時点で、fresh な complete Mission cache があれば same-side を優先して transactional preflight 後に置換する。反対側は既存の tactical reselection/no-return 条件を満たす場合だけ使用する。

## 3. Predictive wall preplan

現在 footprint の 0.30 m warning band に加えて、現在速度と車体方位で短時間先まで前進させた footprint を複数点 sampling する。予測点が warning band に入った場合は hard abort せず、same-side Mission の再評価だけを早期に要求する。現在 footprint の 0.20 m hard wall guard は変更しない。

## 4. Episode logging

Idle から最初の active phase へ入るたびに単調増加する episode ID を割り当てる。phase transition、SafeSeparation、wall preplan の主要ログへ付加し、Progressive/Pass/Recovery の因果を一つの走行ログ内で追えるようにする。

## 非対象

- planner の別 thread 化
- full MPCC / MPPI への置換
- ContactContinuation の攻撃化
- wall hard margin や車体寸法の緩和
