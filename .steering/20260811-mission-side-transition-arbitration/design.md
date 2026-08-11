# Design

## Root cause

現行のno-return latchは`opponent side replan`のcommit経路だけで更新される。
`continuous/scheduled outer transition`は`pass_side_sign`と
`mission_outer_replan_count`を更新するが、同じlatchを更新しない。

このため、個々の機構では左右往復を防いでいても、機構をまたぐと、

```text
initial side 1
  -> continuous outer side -1
  -> opponent replan side 1
```

が成立する。

## Changes

### Mission-wide side-transition latch

`OvertakeLineState`へMission共通のcross-side commit latchを追加する。

- opponent side replacementがsideを変更したらlatchする。
- continuous/scheduled outerがsideを変更したらlatchする。
- latch後は全ての反対side replacementとouter transitionを拒否する。
- Mission resetまで解除しない。
- same-side refreshは対象外とする。

### Complete replacement admission

反対side replacementは、以下をcandidateとprepared Missionの両方で確認する。

- rear-clear rolloutがchecked/feasible
- rear-clear time/distanceが残Mission budget内
- minimum ego speedがtarget speed以上
- rear-clear terminal speedが`target speed + cross-side terminal closing speed`以上
- minimum path wall clearanceが`line minimum + tracking reserve`以上
- candidate自身が追加のfull-track side transitionを必要としない

prepared Missionで不一致が出た場合はcommitせず、旧Missionを保持する。

### Parameter ownership

cross-side terminal speedはPass unlatched capから分離する。

- `v2x_overtake_cross_side_min_terminal_closing_speed`
- `v2x_overtake_cross_side_min_wall_tracking_reserve`

### Dynamic wait preemption

dynamic Mission waitではhard fault、target discontinuity、body overlapを
rear-clearより先に評価する。古いheld pathを安全faultより優先しない。

### Course-progress contract

rolloutのtarget longitudinal position/speedはreference-course progress軸とする。
egoのphysical travelだけをFrenet offset比でreference-course progressへ変換する。
曲率とoffsetを同時に反転した左右対称ケースで同じ結果になることを確認する。

## Failure behavior

- 2回目のcross-side要求は現Missionを維持したまま拒否する。
- prepared gate不成立でも旧Missionを破棄しない。
- hard fault中のdynamic waitはRecoveryへ移る。
