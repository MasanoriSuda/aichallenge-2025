# Design

## rear-clearまでの相手制約

現在の `can_release_receding_horizon_body_clear_bounds()` は、現在車体の非重複と短期予測
の非重複だけで、予測ホライズン全体のtarget boundsを解除できる。これはbody-clearを
rear-clearとして扱う不整合である。

判定をrear-clear契約へ変更し、次をすべて満たす場合だけ解除する。

- Pass phase
- rear-clear confirmed
- target continuity有効
- position jumpなし
- current/predicted footprintが分離
- corridor block、EmergencyBrakeなし

body-clearは速度cap解除の根拠としては残せるが、横経路から相手を消す根拠にはしない。

## Return延期時のPass保持

`begin_validated_return()` がlive Returnを作れなかったことは、現在のPass側も走行不能で
あることを意味しない。rear-clear済みでReturn preflightが延期された場合は、現在の
`e_y` をdistance-domainで保持する短いホライズンを生成する。

このholdは通常の壁余裕で検証し、不成立時だけ設定済みのhard wall余裕で再検証する。
どちらでも実行可能でなければ採用しない。採用時はphase、side、Mission generationを
変えず、次周期にReturn preflightとMPCC-lite再計画を再実行する。

## 非対象

- Frenet DP corridor
- 左右solverの非同期化
- 速度・操舵・加速度を含む完全MPCC

これらは本変更後の動的結果を確認して段階導入する。
