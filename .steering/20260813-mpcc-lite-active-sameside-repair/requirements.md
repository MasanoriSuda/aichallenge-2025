# Requirements

## 目的

MPCC-lite が新規追い越しの入口だけでなく、実行中の同じ側の経路修復にも authority を持ち、target 境界の一時的不整合や近傍 target の分類揺れで即 Recovery へ落ちないようにする。

## 背景

`20260813-190026` では MPCC-lite の entry authority は動作したが、追い越し実行は次の失敗を残した。

- target separation による hard abort: 7 回
- SafetyBrake: 13 回
- `ShiftOut -> Pass`: 9 回に対して `Pass -> Return`: 2 回
- 最終事故前に前方距離が `inf -> 0.01 m` と急変

現行は complete/prefix Mission を選べても、active Mission では `CurrentSideHold` 以外の同側 winner を実行へ反映しない。また、receding-horizon の後処理で target 境界を外れた場合、修復候補を使い切る前に hard infeasible となる経路がある。

## 必須要件

- active ShiftOut / Pass で、MPCC-lite が選んだ complete な同側 Mission を原子的に差し替えられる。
- 同側差し替えに失敗した場合は現Missionを維持し、差し替え失敗だけを理由に Recoveryへ入れない。
- opposite-side差し替えは既存のno-return、full preflight、debounceを迂回しない。
- target execution boundsから外れた後処理結果を同側の実行可能区間へ射影し、物理再検証する。
- 射影修復が成立しない場合、同一target/phase/sideかつhard faultなしならlast-feasible horizonを試す。
- course projectionが一周期失敗しても、位置jumpのない近傍locked targetをlocal geometryで保持する。
- 近傍fallbackは距離と横範囲を制限し、遠方の別コース枝をtargetとして保持しない。
- wall physical contact、EmergencyBrake、position jump、solver recoveryのhard faultは従来どおり優先する。
- ROS 2 topic/service、Domain、評価JSON契約を変更しない。
- ユーザー変更中のDomain 2速度15 km/hと`aichallenge/result-summary.json`を保持する。

## 非対象

- Recovery / Reverse の再設計
- cross-side gate の緩和
- Boost有効化
- `assess_side`の別スレッド実行

## Definition of Done

- same-side authority、near-field target continuityをpure functionでテストする。
- post-validation target-bound failureに射影修復とlast-feasible fallbackを追加する。
- package testと`make autoware-build`が成功する。
- 実走ログでsame-side replacement、target-bound repair、near-field fallbackを識別できる。
