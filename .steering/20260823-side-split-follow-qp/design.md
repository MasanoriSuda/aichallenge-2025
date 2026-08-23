# Side-split Follow QP design (rejected candidate)

## Data-flow correction

Physical row:

`lower <= A_i x <= upper`

を、両sideが有限かつequalityでない場合だけsolver-spaceで次へ展開する。

- lower row: `S_l A_i x >= S_l lower`
- upper row: `S_u A_i x <= S_u upper`

`S_l`と`S_u`は各sideのphysical toleranceから独立に計算する。physical matrix、bound、
post-solve certificateは元の一行を使うため、可行領域とprimal optimumは変わらない。

## Dual mapping

OSQPのlower-side dualは非正、upper-side dualは非負である。

solverからphysical:

`y_physical = S_l y_lower + S_u y_upper`

physical warm startからsolver:

- `y_physical < 0`: lowerへ`y_physical / S_l`、upperは0。
- `y_physical > 0`: upperへ`y_physical / S_u`、lowerは0。
- `y_physical == 0`: 両方0。

非equality rowで両sideが同時activeにはならないため、この符号分解はKKTの意味と整合する。

## Structure ownership

`PreparedProblem`はphysical constraint countとsolver constraint countを分離し、各solver rowに
physical row index、side、scaleを保持する。OSQP workspace構造比較はsolver-spaceで行う。
public `WarmStart`と`SolveResult`のdualは従来どおりphysical-spaceとする。

## Failure-first test

`-1000 <= x <= 0.25`、unconstrained optimum `x=0.4`を用いる。旧row normalizationは
lower側の大きな絶対値でtoleranceが決まりupper違反を許す。side-split policyではupper側だけ
強くscaleされ、`x≈0.25`かつphysical normalized violation <= 1を要求する。

## Authority boundary

実装成功後もFollow shadowだけで使用する。dynamic evidenceなしにproductionへ昇格しない。

## Rejection reason

同じphysical rowを大きく異なるscaleの二行として重複させると、OSQPの内部scalingとglobal
termination下で片側のphysical toleranceは保証されなかった。決定的testではsolverが
`solved`を返した一方、upper違反は0.0895、physical normalized violationは66.8だった。

したがって、制約分割とdual写像が代数的に正しいことだけでは、OSQP terminationとphysical
certificateの一致を保証できない。iteration/tolerance/polishによる救済は行わない。
