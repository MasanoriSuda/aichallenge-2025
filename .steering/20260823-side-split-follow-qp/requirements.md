# Side-split Follow QP requirements (rejected candidate)

## Purpose

Follow shadowで観測した非対称two-sided constraintのsolver toleranceとphysical certificateの
不一致を、上下限をsolver-spaceだけでone-sided rowへ分割して解消する。

## Repaired invariant

各有限bound sideのOSQP-space toleranceは、そのsideのphysical
`eps_abs + eps_rel * abs(bound)`と整合しなければならない。

## Scope

- physical `A/l/u`は変更しない。
- `lower < upper`かつ両方有限のrowだけsolver-spaceで二分する。
- equalityと既存one-sided rowは一つのrowを維持する。
- solver dualを元のphysical rowへ合成して返す。
- physical dual warm startをsolver rowへ符号分解する。
- 新policyはFollow shadowだけへ接続する。

## Non-scope

- OSQP tolerance、iteration、weight、wall marginの調整。
- production authority、publisher、legacy controllerの変更。
- infeasible解のrepair、retry、fallback追加。

## Acceptance

- 非対称boundの決定的testが旧policyでは失敗し、新policyではcertificateを通る。
- mixed-unit、dual、warm-start、workspace reuseの既存testを退行させない。
- full package testを通す。
- Follow shadowのaccepted率を改善し、solve total p95と最大値が25 ms budgetを満たす。
- production authority selected countは0のまま。

## Rollback

staticまたはdynamic gateを満たさなければpolicy接続と実装を同じSliceで削除する。

## Outcome

片側分割のdual/warm-start写像はfocused testを通ったが、mixed-unitかつ非対称boundの
failure-first testで新policy自身がphysical certificateを通せなかった。static gateで棄却し、
全ての候補コード/testを削除した。
