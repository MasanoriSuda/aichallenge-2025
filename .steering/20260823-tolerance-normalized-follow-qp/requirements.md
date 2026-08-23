# Tolerance-normalized Follow QP requirements

## Purpose

mixed-unit extended QPのglobal terminationを、既存row-wise physical certificateと
等価な尺度へ揃える。solver settingやphysical toleranceは変更しない。

## Scope

- constraint rowを既存の`eps_abs + eps_rel * physical_scale`で正規化する。
- `A`, `l`, `u`を同じ正係数で変換し、可行集合を変えない。
- primalは物理単位のまま返す。
- dualはwarm start境界でscaled/physicalを双方向変換する。
- Follow shadowだけへ接続し、未正規化baselineと比較する。

## Constraints

- OSQP settings、physical bounds、certificate toleranceを変更しない。
- solution repair、clamp、retry、fallbackを追加しない。
- legacy MPCとproduction MPCCへ接続しない。
- Follow authorityを昇格しない。

## Definition of Done

- 等価変換前後でphysical optimumと可行集合が一致するunit testを持つ。
- warm-start dualをphysical contractで保存できる。
- dynamic Followでsolver success後のexecution-primal rejectionが減少する。
- accepted rate改善とcallback budgetの両方が確認できなければ不採用とする。
