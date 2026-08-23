# Asymmetric-bound Follow QP audit

## Observation

Follow shadowのphysical certificate rejectは、run `20260823-150821`で22件だった。
21件はstage 0のcurvature/curvature-rate row、1件はstage 0 acceleration rowだった。

## Hypothesis

two-sided rowのcharacteristicを`max(abs(lower), abs(upper))`から、有限sideごとの
toleranceの最小値へ変更すれば、certificateとsolver toleranceが一致する。

## Static falsification

- `make autoware-build`: 成功。
- focused `test_persistent_osqp`: 11/12成功、1件失敗。
- 失敗: `RowToleranceNormalizationClosesMixedUnitToleranceLeak`。
- 例: 物理upper bound 0.25に対して解が約0.397となり、既存のmixed-unit leakが再発した。

`[0, upper]` rowでは厳しい側が常にlower=0となるため、異なる物理単位を持つrow間で
scale差が消える。したがって、単一scaleで厳しい側を選ぶ案は元の目的を満たさない。

## Root cause

一つのtwo-sided solver rowに一つのscaleしか持たせない表現では、lowerとupperで異なる
OSQP toleranceを同時にphysical certificateへ一致させられない。

## Decision

候補を棄却し、コードとtestを全て巻き戻した。dynamic試走、authority昇格、parameter調整は
行っていない。次の候補はsolver境界でtwo-sided rowを二つのone-sided rowへ展開する。

## Next design gates

- caller側のphysical `A/l/u`とcertificateを変更しない。
- solver-spaceだけconstraint数を増やす。
- physical dualとsplit solver dualの写像を明示し、warm startを検証する。
- equality/one-sided rowは不要に複製しない。
- Follow shadow以外へ接続しない。
- runtime 25 msおよび既存回帰testを通らない場合は棄却する。
