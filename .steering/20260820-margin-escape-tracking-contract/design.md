# Design

## 境界契約

dynamic escape candidateには二種類の横境界がある。

1. base wall bounds
   - 通常の車体安全余裕を含むコース境界。
2. selected corridor bounds
   - base wall boundsを車両占有領域でさらに狭めたGapPlanner境界。

`margin-escape` 時、現在横位置がbase wall boundsの外にある側だけ、現在値から
通常境界へ復元距離に応じて線形に収束する継承境界を作る。selected corridorの端が
base wall edgeと一致するときだけ、その端を継承境界まで広げる。selected corridorが
baseより内側なら、その端は車両所有とみなし緩和しない。

これにより、壁余裕だけの初期重複はQPで表現できるが、車両回廊を横切る拡張は
行わない。

## 解の検証

QP成功後、dynamic escapeの予測横軌道を次で検証する。

- QPへ渡したstage境界内であること。
- 実寸footprintのswept pathがstatic wallと非接触であること。

通常のwall clearance marginは復元目標であり、初期escape中のhard footprintへは
追加しない。実寸接触、map外、境界逸脱はsolver failureとして扱い、制御へ渡さない。

## ログ

planning decision traceへ以下を追加する。

- `tracking_contract=<evaluated>/<valid>/<feasible>`
- `tracking_contract_active`
- `tracking_contract_side`
- `tracking_contract_relaxed=<count>`
- `tracking_contract_full=<index>@<distance>m`
- `tracking_contract_max_relax`
- `tracking_contract_first=[lower,upper]/current`
- `tracking_contract_reason`

`tracking_contract_reason` は `inactive`、`already-inside`、
`margin-inherit`、`vehicle-edge-preserved`、`invalid-input`、
`empty-corridor` のいずれかを出す。実寸解検証で落ちた場合はtracking failureの
`reason`へ `dynamic margin escape physical solution rejected: ...` を残す。

tracking failureにはphysical solution validationの理由をそのまま残す。

## 影響範囲

- dynamic obstacle lateral escapeだけを対象にする。
- OvertakeLineのShiftOut/Pass/Return、通常Cruise、通常FollowのQP境界は変更しない。
- runtime Pass wall contractionは同じ物理思想を使うが、Mission再構築側の別契約の
  ため今回の境界継承対象には含めない。
