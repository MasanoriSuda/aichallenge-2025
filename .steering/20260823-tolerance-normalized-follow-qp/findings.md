# Tolerance-normalized Follow QP findings

## 観測された現象

baseline `da6efd9` / run `output/20260823-140735` のFollow shadowは、高速から
前走車へ接近する大部分の周期を解ける一方、mixed-unit QPでsolverが成功扱いした解を
execution-primal certificateが拒否していた。集計は526/603 accepted（87.2%）で、
detail logには35件のexecution-primal rejectがあった。

## 問題が発生するまでの因果関係

1. 5-state QPは横位置、姿勢、速度、進捗、入力rateを一つのconstraint matrixへ持つ。
2. OSQPのglobal termination scaleは10--20 m級の進捗行に支配される。
3. acceleration、curvature、rateなど小さい物理単位の行が、その行固有の許容差を
   超えてもsolver全体はsolvedになり得る。
4. 後段のphysical execution certificateが初めて違反を検出し、Follow shadowを拒否する。

見えていたexecution-primal rejectは根本原因ではなく、mixed-unit global termination漏れを
検出した下流guardだった。

## 根本原因

同じQP内の異なる物理単位を、OSQPへ未正規化のまま渡していたこと。安全意味ごとの
row toleranceはpost-solveでしか使われず、solverの収束尺度と一致していなかった。

## 根拠

- deterministic test `RowToleranceNormalizationClosesMixedUnitToleranceLeak`では、同じQPに
  0.25と1000.0のboundを置くと未正規化solverが小さい行へ0.397を返した。正規化solverは
  0.250000を返し、row-wise certificateを満たした。
- row 270はN=20 extended QPのfirst curvature-rate constraintに対応する。
- strict scaled-termination案のrun `output/20260823-142145` は241/788 accepted（30.6%）へ
  悪化したため、OSQP内部scaleでの終了判定切替は不採用とした。
- provisional inverse-tolerance transformation `S_i=1/t_i` のrun
  `output/20260823-143939` は502/512 accepted（98.0%）、execution-primal reject 0件だった。
  Follow solveのattempt-weighted平均は2.31 ms、最大30.97 msだった。ただし、この式はOSQPの
  global absolute toleranceまで同時に縮小し、zero-bound行を意図より厳しくするため不採用とした。
- exact tolerance-preserving transformation `S_i=T/t_i` のrun
  `output/20260823-144839` は1305/1322 accepted（98.7%）、execution-primal reject 0件だった。
  Follow solveのattempt-weighted平均は2.56 ms、最大30.87 msだった。停止target 0 m/sを含む
  連続windowも3/3、41/41、41/41、6/6 acceptedだった。
- baseline `output/20260823-140735` は526/603 accepted（87.2%）、execution-primal reject
  35件、attempt-weighted平均4.41 ms、最大35.95 msだった。

run間でtrafficとproduction overtake状態は完全同一ではない。全callbackのoverrun比較には
start-grid側production solver failureが混入するため、今回の採否はFollow shadow固有の
accepted/solve telemetryに限定する。

## 既存パッチとの関係

global absolute residual checkとexecution-primal certificateは削除しない。前者はdefault
solverの互換契約、後者は物理単位のhard oracleである。今回の正規化はそのguardを回避せず、
solverへ渡す等価問題の尺度をguardと揃える。

## 修正方針

各有限bound行の既存許容差`t_i`と、その最大値`T=max(t_i)`から`T/t_i`を求め、`A`,
`l`, `u`を同じ正係数で変換する。共通係数`T`を含めることでOSQPのglobal absolute
toleranceを物理座標へ戻した値も既存row certificateと一致する。可行集合とprimal optimumは
変えない。dualはsolver境界でscaled/physicalを変換し、callerとwarm-startは従来どおり
物理constraint座標だけを扱う。

## 実施した変更

- `ConstraintPreconditioningPolicy::RowToleranceNormalized`をtyped solver policyとして追加。
- physical constraint matrixとsolver用scaled matrixを分離。
- warm-start入力をphysical dualからscaled dualへ変換し、solver出力をphysical dualへ戻す。
- normalized policyでは元のphysical `A,l,u`によるrow certificateを必須化。
- Follow shadow専用contextだけへ接続。production Track/Cruise/Overtake authorityは未変更。
- mixed-unit tolerance漏れと、scaleが周期間で変わるdual warm-startのunit testを追加。

原因と変更の対応は、mixed-unit termination漏れに対してrow-equivalent transformation、dual
coordinate不整合に対してsolver境界の双方向変換、の2点である。

## 削除・整理できた処理

production branchは削除していない。このsliceは明示的なshadow-only measurementである。
先行実験で追加したscaled termination / polish / row-certified wrapperは悪化を確認して既に
削除済みで、feature flagとして残していない。

削除milestoneはFollow authority昇格sliceである。その時点で同じcanonical solver policyへ
統一し、Followのlegacy longitudinal ownerを同じsliceで削除する。昇格しない場合はこのpolicy
接続自体を削除する。

## 残っている懸念

- final run `144839`では17/1322周期が不採用だった。throttled detailに残った14件はすべて
  solver status `solved`後のphysical row certificate拒否で、normalized violationは
  1.00043--1.66317だった。row 270が10件、143/148/212が各1--2件であり、混合単位の
  global tolerance漏れとは分離された収束境界の問題である。guardの閾値は緩和しない。
- Follow solve最大30.87 msは40 Hzの25 msを超える。平均は改善したがproduction authority
  昇格にはworst-case縮減または非同期化の証拠が必要である。
- stopped-targetは今回runで連続成立を確認したが、異なるgap/速度/seedを含む反復証拠は
  まだ不足する。

## 次回試走で確認すべき項目

- target gap 2--6 m、ego 7--9 m/sのFollow shadow accepted率。
- max-iteration最初の周期におけるstage geometry、hard progress upper、velocity upper、
  curvature-rate rowの値。
- stopped targetでのaccepted率とsolve p95/p99/max。
- stale resultやauthority昇格が0であること。`authority=shadow, selected=0`を維持すること。

## 判定

Follow shadowでの継続採用は可。Follow production authorityへの昇格は不可。残る物理行証明の
境界超過を別root-cause sliceで監査し、25 ms worst-caseと停止対象の反復証拠を満たすまで
legacy Follow ownerを削除しない。
