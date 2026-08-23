# Follow row provenance audit findings

## 観測された現象

run `output/20260823-150821`ではFollow shadow 1275 attempts中1253 accepted
（98.27%）、22件がphysical row certificateで拒否された。

typed集計は次だった。

- curvature-rate / curvature / stage 0: 21件
- input-box / acceleration / stage 0: 1件
- state-box / velocity: 0件
- input-box / virtual-progress-speed: 0件
- その他: 0件

Follow solveはattempt-weighted平均2.43 ms、最大22.24 msで、今回runでは25 ms以内だった。

## 問題が発生するまでの因果関係

1. 一つのconstraint rowへ有限lower/upperの両側を持たせる。
2. preconditionerは`max(abs(lower), abs(upper))`をrow characteristicに使う。
3. 非対称boundでは、絶対値の大きい側からsolver-space許容差が決まる。
4. 解が絶対値の小さい側を破ると、physical certificateはその実値・projected boundから
   より小さいrow toleranceを計算する。
5. OSQPはglobal criterionを満たして`solved`を返すが、physical certificateは同じ解を拒否する。

例としてstage 0 accelerationは、bound `[-3.0, 1.37]`に対して解が`1.37258`だった。
preconditionerのcharacteristicは3.0だが、上限側certificate toleranceは0.00237258であり、
violation 0.00257943を拒否した。

curvature-rateも同じである。例ではbound `[0.144321, 0.176523]`に対して値0.143152、
violation 0.00116901、lower側tolerance 0.00114432だった。21件は正負両方向に発生し、
いずれも実際に破った側がrow characteristicに採用されなかったケースだった。

## 根本原因

残存拒否の根本原因はFollow幾何、前走車予測、wall marginではない。二側boundを一つの
row scaleで表す際に、緩い側の許容差を採用していたため、solver terminationとphysical
certificateが非対称boundで一致していなかったことにある。

## 既存パッチとの関係

`481949d`の`T/t_i`変換はmixed-unit global tolerance漏れを大幅に削減したが、`t_i`を
二側の最大絶対値から一つだけ作る点が残っていた。post-solve certificateや下流guardは
正しく不一致を検出しているため削除・緩和しない。

曲率rate boundの`tan(rate*dt)/wheelbase`近似にもモデル誤差はあるが、今回の21件は
どちらのboundを置いてもactive sideとscaleが不一致なら再発し得る。まずsolver/certificate
契約を揃え、その後に操舵角rateから曲率boundへの変換精度を別Sliceで監査する。

## 次の修正Gate

次のshadow実験では、各rowの有限lower/upperそれぞれのphysical toleranceを計算し、より
厳しい側をrow scaleに採用する。これはphysical feasible setを変えず、solverを緩い側で
必要以上に厳しくする可能性だけを持つ。動的Gateでruntime悪化が大きい場合は、上下限を
別rowへ分離する正確だが高コストな案と比較する。

禁止事項は継続する。OSQP tolerance変更、certificate緩和、解のclamp、retry、production
authority昇格は行わない。
