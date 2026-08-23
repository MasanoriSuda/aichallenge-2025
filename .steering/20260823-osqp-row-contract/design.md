# Design: OSQP row-contract root-cause audit

## Observed causal chain

Valid run `output/20260823-081219` では、OSQP solve自体は成功している一方、
実行境界検査が主にstage 0の曲率を拒否した。例:

- QP曲率上限相当: 約 `0.35314 rad/m`
- OSQP primal: `0.363866 rad/m`
- row violation: `0.0107256`
- row tolerance: `0.00136387`

`PersistentOsqpSolver` はrow別residualを計算しているが、solve採否はQP全行の
最大値から作った単一のglobal toleranceを使用している。進捗・速度など大きい単位の
行が同居するため、小さい曲率行が自身のrow toleranceを超えてもglobal checkを通る。
後段のnormalizerだけがrow契約を適用するため、`solved -> execution-primal-reject ->
emergency stop` が発生する。

## Hypotheses

### H1: global relative tolerance leaks across physical units

- Support: row violation > row toleranceなのにsolver wrapperがresultを返している。
- Refutation: rejected cycleでmaximum normalized row <= 1、または別の境界定義が使われている。
- Confidence: high.

### H2: explicit row scaling can align OSQP termination with the execution contract

- Support: QP rows mix metres, radians, m/s, rad/m, progress metres.
- Refutation: scalingしても同カテゴリのrow violationとmax-iteration率が改善しない。
- Result: partially supported numerically, rejected as a production change.
  Row reject rateは大きく低下したが、wall/contact proof reject、max iteration、
  callback overrunを悪化させた。

### H3: warm start is the primary source

- Support: most rejected cycles are warm-started.
- Refutation: cold cyclesでも同じrow categoryが拒否される、またはrow contract mismatch aloneで再現できる。
- Result: not primary. Row scaling後のdual座標を次周期scaleへrebaseしても、
  warm solveのrow-contract拒否は残った。

## Candidate comparison

### A. Unscaled QP with `eps_rel=0`

`output/20260823-093013` で、solve時間が約22 msへ増加し、4000 iterationと
callback overrunを開始前から反復した。mixed-unit QPを絶対許容誤差だけで解く案は
リアルタイム契約を満たさないため撤回した。

### B. Per-row constraint normalization

各有限制約行を、その行の
`eps_abs + eps_rel * max(abs(lower), abs(upper))` で正規化した。
可行集合は変えず、OSQPのglobal stopping normで単位間の許容誤差を借用しない狙いだった。

`output/20260823-093759` では、baselineのproduction execution reject
`26 / 3910 decisions` に対してrow-contract rejectは `4 / 3589` まで減った。
一方でwall/contact proof reject 7件、stuck confirmed 4件が発生したため未採用とした。

### C. Row normalization plus scaled-dual provenance

constraint rowをscaleするとdual座標も変わる。物理Lagrange multiplierを保存するため、
warm start dualを `old_scale / new_scale` でrebaseする候補も実装・試験した。

`output/20260823-095004` の1周超では:

- production row-contract reject: `5 / 5559 decisions`
- maximum iteration: 4
- wall/contact proof reject: 16
- callback overrun: 1
- confirmed stuck: 0

row rejectはbaseline比で約86%減ったが、Definition of Doneで禁止した物理証明拒否、
max iteration、overrunの悪化が生じた。dual provenanceは数学的には必要だったが、
本症状の主因ではなく、この候補全体を撤回した。

## Root-cause conclusion

global relative toleranceがmixed-unit row間で漏れることは確認済みである。ただし、
solver入力を後付けでrow正規化するだけでは、OSQPの数値軌道と実行trajectoryが変わり、
現在のwall certificate / first-stage model不整合を露出または悪化させる。

したがって次に必要なのは、許容誤差だけの局所修正ではない。5状態MPCC内部の
state/input nondimensionalization、first-stage実行モデル、wall certificateを同じ座標・
時間基準で監査してから、solver acceptanceを一本化する必要がある。

## Investigation order

1. constraint layoutをcategory/stageへ分類する純粋関数を作る。
2. global採否とrow別契約の不一致をfailure-first testで固定する。
3. QP row scalingまたはsolver termination policyを比較する。
4. 最小で整合的な案だけを実装し、旧global-only acceptanceを削除する。
5. static test後に1周の無人試走で動的証拠を取る。

## Non-goals

- downstream primal restoration
- tolerance緩和
- max iteration増加
- wall/racing parameter調整
-新しいfallbackやauthority leaseの追加
