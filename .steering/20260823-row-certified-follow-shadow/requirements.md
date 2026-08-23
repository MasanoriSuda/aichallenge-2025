# Row-certified Follow shadow requirements

## Purpose

Follow shadowで観測した、OSQPのglobal infinity-norm終了条件とrow-wise execution
certificateの不一致を解消し、solver successの意味を実行採用条件と一致させる。

## Scope

- Persistent OSQPへ型付きnumerical contractを追加する。
- Follow shadowだけをrow-certified contractへ接続する。
- mixed-unit QPではscaled terminationとsolution polishを使用する。
- row-wise normalized violationが1を超える解はsolver successとして返さない。
- 設定未変更の`make dev3`でaccepted rateとsolve timeを比較する。

## Constraints

- eps_abs、eps_rel、最大反復数を変更しない。
- execution certificateのtoleranceを緩めない。
- retry、fallback、runtime flagを追加しない。
- Followをproduction authorityへ昇格しない。
- legacy MPCとproduction MPCCのsolver modeを変更しない。
- wall、gap、velocity policyのparameter tuningを行わない。

## Definition of Done

- row-certified solverがsuccessを返すとき、全constraint rowが既存certificate内である。
- Follow shadowで`execution-primal-reject`がsolver成功後に現れない。
- accepted rate、maximum-iterations、solve p95/maxへの影響を記録する。
- 効果が不十分なら設定を追加せず、formulation conditioningを次原因として残す。
