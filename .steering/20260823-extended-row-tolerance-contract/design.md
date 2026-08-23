# Design: extended MPCC row tolerance contract

## Root-cause audit

### Rejected hypotheses

1. `BicycleModel::t2s()`とcanonical Frenet投影の横座標式が異なる
   - 両方とも同じcourse waypointを基準に、
     `cos(psi)*dy - sin(psi)*dx`で横位置を計算する。
2. 左右非同期branchがlive modelのspatial stateを破壊する
   - branch評価は`BicycleModel`と`ReferencePath`のsnapshotを複製して実行する。
3. 表示された`stage=0`が現在状態の二重投影不整合を示す
   - 横制約監査は`(stage + 1)`のbox rowを走査しており、表示stage 0はx1である。

### Root cause

5-state extended solverのproduction/default contextは
`ConstraintPreconditioningPolicy::None`を使っている。

このpolicyではsolver後の受理判定が、全constraint valueの最大絶対量から作る
単一のglobal toleranceに依存する。MPCCには横位置[m]、速度[m/s]、進捗[m]、
各dynamics rowが混在するため、大きいrowのscaleが横box rowの許容範囲まで
実質的に拡大する。その結果、横row固有の許容差を超えた解がsolver全体では
成功扱いされる。

Follow canonical solverだけは既に`RowToleranceNormalized`を使用している。
このpolicyは各rowの物理許容差を基準にpreconditionし、受理判定も
`maximum_normalized_violation <= 1`とするため、mixed-unit contractと一致する。

因果関係は次の通り。

`5-state mixed-unit QP`
→ `global-scale solver acceptance`
→ `x1 lateral box rowが約6-10 cm逸脱してもsuccess`
→ `legacy conversionは利用可能`
→ `canonical physical contractが後段で棄却`
→ `同じ解に対するauthority経路の判断不一致`

## Rejected broad correction

最初に全5-state solver contextを`RowToleranceNormalized`へ統一したが、
Track-only replayでTrack/Cruise productionが45周期solve-failureとなった。
主にdynamics row 210で`maximum_normalized_violation > 1`となり、retained候補も
dynamic obstacle presenceで使えずEmergency Stopへ落ちるため、本sliceの範囲では
退行である。

Track/Cruiseの行別収束契約まで直すには、別のfailure-first sliceでdynamics rowの
定式化・scaling・warm startを監査する必要がある。Overtakeの横row不整合修正へ
同問題を混ぜない。

## Chosen correction

後段で行別物理証明を要求するOvertake/Follow contextへ
`RowToleranceNormalized`を明示的に適用する。

- Overtake/DynamicEscape live extended solver
- left/right tactical branch solver
- Follow canonical solver（既存契約を明示的に維持）

Track/Cruise contextは従来のpolicyを維持する。これは恒久的に異なる成功定義を
正当化するものではなく、既にproduction昇格済みのauthorityへ別原因の退行を
混入させないslice境界である。

この修正は閾値調整ではなく、Overtake候補のsolver成功定義を既存の行別物理制約
契約へ合わせる構造修正である。新しいflag、fallback、timeoutは追加しない。

## Alternatives rejected

- 横boundを広げる: 症状を隠し、wall/corridor契約を弱める。
- shadow側だけglobal toleranceを許容する: canonical authorityが物理契約違反解を採用する。
- 解を横boundへclampする: dynamics equalityを壊した未証明trajectoryになる。
- lateral rowだけ後段で棄却する現状維持: legacyとcanonicalで成功定義が分裂したまま。

## Expected effect

- lateral row violationがrow tolerance以内ならshadow chainへ進む。
- 以内へ収束しない周期はsolver failureとして一貫して縮退し、不正解をlegacyへ渡さない。
- preconditioningにより収束性が改善する可能性があるが、replayでsolve timeとfailureを必ず確認する。

## Remaining boundary

Track/Cruiseで行別正規化をproduction採用するには、row 210を中心とするdynamics
constraintのfailure-first replayを別途用意し、Emergency Stop増加なしを証明する。
本sliceではその問題をNone policyで隠したのではなく、既存authorityの契約を変更せず
監査対象として分離した。
