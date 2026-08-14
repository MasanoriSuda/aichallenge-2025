# Design

## 背景

直近走行では、DynamicMissionWaitのforward prefixはMission closing speed 2.0 m/sを
正常に発行していた。一方、同じ周期群でBehavior側はFollowPrepareをcommitted executionと
認識せずSafetyBrakeへ遷移し、再開したPassも`pass_front_cap_release_active=false`から
0.5 m/s closingへ戻っていた。

## 方針

### 1. Forward authorityの明示

pure coreに`DynamicMissionWaitForwardAuthorityRequest`を追加する。前周期に壁検証済みの
full-closing prefixが成立し、現在もtarget continuity、現在車体非重複、予測有効、予測sweep
非重複を満たす場合だけforward authorityを有効とする。

### 2. Behavior仲裁

通常のShiftOut/Passに加え、上記forward authorityを
`can_suppress_committed_corridor_front_danger()`のexecution authorityとして認める。
fixed corridorおよび既存のtarget/geometry guardは引き続き必須とする。

### 3. Pass再開handoff

DynamicMissionWaitからfresh same-side Pass continuationへatomic replacementするときだけ、
forward authorityを`pass_front_cap_release_active`へseedする。side変更、Pass continuation以外、
現在geometry不成立ではseedしない。

## ログ確認点

- same-side replacementログの`front_cap_handoff=1`
- その直後のPass debugで`cap_release=1`
- DynamicMissionWait full prefix中の`front_danger_suppress=1`
- current overlapまたはprediction invalid時は上記が0となること
