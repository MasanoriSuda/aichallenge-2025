# Asymmetric-bound Follow QP requirements (rejected candidate)

## Purpose

Follow shadow row preconditioningを、非対称lower/upperの緩い側ではなく、physical certificateと
整合する厳しい側のtoleranceへ揃える。

## Scope

- 各有限boundの`t=eps_abs+eps_rel*abs(bound)`を個別に計算する。
- 一つのtwo-sided rowでは最小の`t`を採用する。
- 既存`T/t_i`変換、dual変換、physical certificateは維持する。
- Follow shadow専用policyだけで動的比較する。

## Constraints

- physical feasible set、bound値、OSQP setting、certificate toleranceを変更しない。
- clamp、repair、retry、fallback、parameterを追加しない。
- production authorityへ昇格しない。
- runtimeまたはaccepted率が悪化した候補は削除する。

## Definition of Done

- asymmetric、zero-crossing、one-sided、equality boundのpure testを持つ。
- run `150821`で観測したacceleration/curvature-rate不一致を式上再現できない。
- dynamic Follow shadowでrow rejectが減り、25 ms budgetを悪化させない。

## Outcome

この候補は静的gateで棄却した。two-sided rowに単一scaleを適用する限り、上下限ごとに異なる
physical toleranceを同時には表現できない。厳しい側のtoleranceを選ぶと、`[0, upper]` のrowは
ゼロ側に支配され、mixed-unit row normalizationの効果を失う。
