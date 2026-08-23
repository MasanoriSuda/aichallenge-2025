# Design

## Root cause

壁境界は `ref_wp_id + stage` で生成され、extended QPの各stageへ固定box rowとして渡される。一方、5-state MPCCはcourse progressを状態として最適化し、同じstageで基準progressから前後へ移動できる。

物理証明は正しくsolved progressでcourse frameを再構築する。このため、QPは別地点の壁境界を使って可行と判定し、物理証明だけが実際の地点の壁接触を検出する。

再現例:

- stage=5
- reference progress=36.759 m
- solved progress=30.789 m
- delta=-5.970 m
- QP lateral=1.581 m, bounds=[-0.044,1.699] m
- QP reserve=0.118 m
- actual footprint wall contacts=2

heading offsetも物理footprintへ影響するが、まず6 mの位置対応誤りを解消しなければならない。

## Rejected alternatives

1. wall marginを増やす: 位置対応誤りを隠すだけで、別区間で再発する。
2. progress trust regionを狭める: MPCCの進捗最適化を事実上legacy化する。
3. physical certificateを緩める: 壁接触解をproductionへ通すため不可。
4. retained Overtakeをageだけで採用する: 現在世界での壁可行性を証明しない。

## Rejected implementation experiment

最初の実装では、first solveのprogressでwall boxを引き直し、同じQPを1回だけ
warm-start solveした。しかし同一bag replayでは次が観測された。

- refinementはDynamic Escapeで32 telemetry windowすべて作動した。
- first solveのstage progress差は最大 `17.093 m`、profile範囲外は合計290 stage。
- refined solve後にも `HardWallContact` が残り、集約件数は旧版5件に対して7件。
- solve failureは増えなかったが、solve時間とiterationは増加した。

原因は、wall boxを更新するとsecond solveが別のprogressへ移動できるためである。
「一度解いた位置のboxへ差し替える」方法はprogressとwallを同じ最適化変数として
結合しておらず、根本原因を解消しない。この実験はproductionへ採用しない。

## Implemented structure

extended Overtake solveへ、lateralとprogressを結合したwall rowを追加する。

1. first solveでprogress trajectoryを得る。
2. 各stageのsolved progressを含むwall profile区間を特定する。
3. 区間内のwall上下端をprogressの一次式として表す。
4. `e_y - slope * theta` の上下2 rowをQPへ追加する。
5. 同じstageのthetaを、その一次式が有効なprofile区間内へ制限する。
6. first solutionをwarm startに、同じdecision/contextでrefined solveを行う。
7. refined solutionを従来の実車体・実地図証明へ通す。

profileは現在位置の `progress=0` から持たせる。空間horizonの最初のwaypointより
手前にいる低・中速のtemporal horizonを範囲外として放置しない。

区間内ではwall rowとthetaが同じQP変数として結合されるため、refined solveが
progressを動かしても、別地点のwall boxを使うことはできない。

再線形化が構造的に作れない場合、first solutionは従来どおり最終実車体証明へ進み、
そこで壁接触解を棄却する。refined solveが失敗した場合も同じであり、未証明解を
productionへ通す新fallbackは追加しない。

QPの疎行列構造を周期中に変えないため、`2 * N`本のwall rowは全extended problemへ
常設し、対象context以外では上下限を無限として休止する。Overtake/Dynamic Escapeでは
first solve後に係数と上下限だけを更新する。Followの末尾gap rowとwall rowは別blockとして
dual warm-startのshift契約へ明示する。

## Scope guard

- wall rowの実制約化はOvertake canonical fresh shadowとDynamic Escapeが使うextended
  problemに限定する。Track/Cruise・Followではrowを休止し、既存authorityを変えない。
- target obstacleのstage-time制約はこのSliceでprogress補間しない。
