# Tolerance-normalized Follow QP design

## Transformation

各constraint row `i`について、有限boundの代表scaleを

`b_i = max(abs(l_i), abs(u_i))`

とし、既存certificate tolerance

`t_i = eps_abs + eps_rel * b_i`

を作る。全有限bound行の`T = max(t_i)`を求め、

`S_ii = T / t_i`

とする。両側無限のrowは`S_ii = 1`とする。

この共通係数`T`は省略してはいけない。OSQPのglobal absolute toleranceもsolverへ渡す
問題の尺度で評価されるため、`1 / t_i`だけでは全rowを過剰に拡大し、特にzero-bound行を
既存certificateより厳しく解かせる。`T / t_i`ならscaled problemのglobal toleranceが`T`に
なり、物理座標へ戻した各rowの許容差は`T / S_ii = t_i`となる。

solverへ渡す問題は

`S A x in [S l, S u]`

であり、元の問題と可行集合・primal optimumは同じである。

## Dual provenance

scaled dualを`y_s`、physical dualを`y_p`とすると、stationarityより

`y_p = S y_s`

である。solver入力warm startは`y_s = S^-1 y_p`、solver結果保存時は
`y_p = S y_s`へ戻す。

## Safety contract

solver結果は元のphysical `A,l,u`で再検証する。row-wise normalized violationが1を
超える場合はresultを返さない。certificate toleranceは従来値のまま。

## Migration boundary

Follow shadow solver contextだけがこのpreconditioningを使う。他authorityの数値挙動は
動的Gate通過まで変更しない。
