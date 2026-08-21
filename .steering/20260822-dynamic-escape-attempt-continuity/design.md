# Design

## 方針

ノード本体に直書きされたattempt ID管理をpure C++の
`DynamicEscapeAttemptTracker`へ切り出す。attemptはsolver呼び出し単位ではなく、
同じ前方targetとのencounter単位として扱う。

## 状態遷移

- inactive + request + relevant target: 新規attempt開始
- active + same relevant target + request: 同一attempt継続
- active + same relevant target + no request: 同一attemptを保持
- active + target一時欠落: `target_loss_grace_sec`内は保持
- active + target欠落timeout: attempt終了
- active + target変更:
  - 新targetへのrequestがあればatomic retargetして新attempt開始
  - requestがなければ旧attemptを終了
- race session reset / explicit reset: attempt終了

## 安全境界

attempt continuityはIDと戦術文脈だけを保持する。以下は従来どおり独立して期限切れにする。

- retained control lease
- wall admission
- physical execution certificate
- solver result age
- target prediction / footprint判定

## ログ

`Dynamic escape attempt lifecycle`を追加する。

- event: started / retargeted / heartbeat / released
- reason: planner-requested / request-gap-held / target-loss-grace / target-lost / target-changed
- attempt、target、planner request、target relevance
- lifetime cycle、request cycle、gap cycle、target-loss age/grace

開始・retarget・終了は即時、通常保持は低頻度heartbeatだけを出力する。
