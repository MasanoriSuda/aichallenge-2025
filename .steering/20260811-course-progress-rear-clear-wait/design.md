# Design

## 1. Dynamic Mission wait

`enter_dynamic_mission_wait()` は、失敗したfrozen path generationの復活を防ぐため現Missionをinvalidateする。このinvalidateはwaitの正常な開始状態であり、replacement未成立を即Recovery理由にしてはいけない。

policyを次に統一する。

- hard fault / target invalid / body overlap: Recovery
- rear-clear: Return
- fresh alternate replacement: ReplaceWithAlternate
- fresh current replacement: ReplaceWithCurrent
- assessment前: Hold
- invalidate済みでreplacementなし: Hold
- invalidateされていないcurrent feasible: ResumeCurrent
- それ以外: Hold

期限は既存の `0.75 s / 4 m` が所有する。

## 2. Course-progress-aware rear-clear rollout

現行rolloutは物理速度をそのままreference course distanceへ積分する。Frenet offset上では、reference progressは概ね次となる。

```text
s_dot = v / (1 - kappa * e_y)
```

各speed-cap sampleへ `course_progress_ratio` を追加し、既存のoffset curve feasibilityが返すFrenet denominatorの逆数を保存する。

rolloutでは物理走行距離を維持したまま、targetとの相対course progressだけをこの係数で積分する。

- outer offset: denominator > 1、progress ratio < 1
- inner offset: denominator < 1、progress ratio > 1

これによりヘアピン外まくりのrear-clearを楽観評価しにくくする。lateral ShiftOut距離、速度cap、壁preflightのauthorityは変更しない。

## Verification

- invalidated generationがreplacement待ちを継続するpure policy test
- invalid/zero progress ratioをfail-closedにするtest
- outer/innerのrear-clear時刻差を確認するrollout test
- `make autoware-build`
- package test
