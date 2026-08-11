# Design

## Root cause

no-returnは毎周期、`target_front_distance >= threshold`で再計算される。そのため一度並走領域に入っても、egoが失速しtargetが前へ離れると反対sideが再許可される。

SafeSeparationのlast-feasible alternate rescueはこの現在距離だけを用い、committed-state blockの例外としてPass後半の全幅切り返しを許可していた。

さらにPass中のside置換はphaseをPassに保ったままfront-cap latchを解除する。これはShiftOut時のclosing-speedで予測したcandidateを、runtimeではPass-unlatched capで走らせる不整合になる。

## Changes

### 1. Monotonic no-return latch

Mission中に以下のいずれかを一度観測したら、cross-side no-returnをMission終了まで保持する。

- lateral/front-overlap exclusion latch
- forward-completion latch
- SafeSeparation開始
- targetが縦方向no-return距離内
- 反対side置換のcommit成功

### 2. Pure completion admission

cross-side candidateについて以下をpure policyで確認する。

- no-return/committedでない
- candidateがfeasible
- rear-clear rollout checked/feasible
- rear-clear時間と距離が残りabsolute Pass budget内
- 予測最低速度がtarget速度以上
- rear-clear時の速度がtarget速度 + unlatched closing margin以上

同側refreshはこのcross-side専用gateの対象外とする。

### 3. Restart ShiftOut

Pass初期にcross-side replacementをcommitした場合は、Pass内部のlateral replanではなくShiftOutへ戻す。これによりcandidateとruntimeのclosing-speed stageを一致させる。

## Failure behavior

cross-side candidateがgateを通らない場合は旧Missionを保持する。SafeSeparation中なら同側前進/分離を継続し、置換失敗自体でRecoveryに落とさない。

## Verification

- no-return latchがtargetの再前進で解除されない。
- SafeSeparation中のalternate rescueが拒否される。
- 残りtime/distance budget超過のcandidateが拒否される。
- 速度条件不足のcandidateが拒否される。
- 許可されたPass初期の反対side置換がShiftOutを再開する。
