# Design

## 1. Active same-side authority

MPCC-liteのwinnerが現在sideと同じLeft/Rightで、complete Missionを持ち、target continuityとruntime hard fault条件を満たす場合は`ReplaceActive`を返す。適用対象は通常の`ShiftOut` / `Pass`に限定し、start-grid breakout、FollowPrepare、Return、Recoveryには適用しない。

controllerはwinner Missionを`V2XBehaviorOutput`へ渡し、OvertakeLine更新側で既存のtransactional Mission replacementを使う。これによりfreeze、generation、pass進捗、rollback処理を重複実装しない。

差し替えcommitが拒否された場合は現在Missionを保持する。差し替え前にfront-cap releaseが成立済みならその状態も保持し、5 Hzの再計画が並走中の再失速を起こさないようにする。反対側winnerは従来のopponent-side replan gateへ任せる。

## 2. Target-bound projection repair

receding-horizon optimizer後のstatic-map・横加速度補正でtarget execution boundsを外れた場合、各sampleを次の共通実行区間へ射影する。

1. wall lower/upper
2. 現在pass sideに対するtarget center separation
3. hard wall reserve

射影した系列を再度`evaluate_overtake_line_horizon`へ通し、車体footprint、壁、横加速度、target boundsを再検証する。物理的に共通区間がない場合は修復しない。

全候補が不成立でも、同一contextのwarm startがexecution lease内ならlast-feasible horizonを最後に試す。hard fault時はleaseが成立しない。

## 3. Near-field locked-target continuity

course progress projectionはヘアピンの近接枝で一時的に不成立になり得る。locked targetについてのみ、次の条件でlocal vehicle frameへfallbackする。

- position jumpなし
- local longitudinal/lateral/self distanceがfinite
- self distanceとlongitudinalが設定したnear-field距離以内
- lateralが現在のcourse corridor範囲内

fallback中はlocal longitudinal/lateralをlocked-target geometryとして使い、course progress rejectionをhard discontinuityとして扱わない。範囲外では従来どおりfail closedとする。

## 4. 非同期化境界

現在の計測時間の大半はpure scoreではなく`assess_side`内のpath/corridor/rollout生成である。この処理は`model`、Mission state、wall gridを参照するため、そのままthreadへ移すと競合する。

次段では次をimmutable snapshotへ抽出する。

- reference path sampleとwall bounds
- ego pose/speedとMission identity
- targetの時系列予測
- left/right goal候補とbudget

workerはlatest-only queue size 1でsnapshotを受け、main controlは待たず、target/generation/phase/sideとageが一致する結果だけ採用する。本変更ではその前提となるactive same-side authorityとfail-safe replacementを完成させる。

## 5. ログ

- `authority=replace`かつsame-side winner
- `MPCC-lite same-side PassPlan replaced/rejected`
- `post-validation trajectory projected to execution bounds`
- `post-validation failed; retained last feasible`
- 既存1 Hz behavior debugの`near_field=1/0`

周期ログは既存の1 Hz制限を維持する。
