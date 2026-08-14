# Design

## 原因

GapPlannerは30 m先まで計算しているが、従来は`target_active=true`の点だけを
`FrenetDpCorridorRequest`へ渡していた。V2X予測は1秒なので、実走ではtarget footprintと
同じ縦断面になる5点、約7〜10 mだけがDP列となった。

その結果、DPはShiftOut前半では有効でもPass完了前にcoverageを失い、旧単一goal参照へ
戻った直後にtarget hard boundと衝突していた。

## DP profileの分離

動的Mission検証に使う`dynamic_mission_corridor_samples`は従来どおり
`target_active=true`だけを保持する。これによりprediction expiryや動的valid distanceの意味を
変えない。

別に`frenet_dp_execution_corridor_samples`を作り、同じGapPlanner horizonの全点を格納する。

- target active点: GapPlannerが選んだ車両・壁間corridorをそのまま使用
- target inactive点: base wall boundsをrobust planning clearanceで縮小して使用
- corridorが物理的に消える点: そこでDP extensionを打ち切る

DPのmotion costにより、障害物前は現在横位置を維持し、障害物区間で選択sideへ移り、
障害物後は不要な横移動を行わず同じ側を維持する。Returnは従来どおりrear-clear確認後に
別profileへ切り替える。

## Rolling refresh

active ShiftOut/Pass中に、既存opponent-side評価が生成したcurrent-side MissionからDP列だけを
取り出す。次を全て満たすときだけ更新する。

- feature enabled
- active phaseがShiftOutまたはPass
- active target IDとBehavior target IDが一致
- candidate sideがactive sideと一致
- candidate predictionがfresh
- candidate生成時刻が直前DP sourceより新しい
- 最小更新間隔を経過
- DP距離列・横位置列がvalid

commitはDPのside、距離列、横位置列、source時刻をまとめて置換し、DP traveledを0へ戻す。
Mission generation、Pass累積距離、front-cap latch、Return位置は変更しない。

更新候補がない周期は既存DP列とtraveledを保持する。これはlast-feasible leaseとして働くが、
target continuity、wall、EmergencyBrake等の既存hard guardは引き続き優先される。

## ログ

- freeze: `dp_execution=1/<points>/<distance> m`
- refresh: target、side、点数、距離、旧remaining、refresh count
- execution: active、covered/N、remaining、refresh count/age
