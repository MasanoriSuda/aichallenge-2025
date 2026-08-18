# Design

## 1. Static wall component cache

従来のcache keyは`lower/upper/preferred`と非常に細かいheadingを含み、動くMission targetを
そのまま静的地図cacheへ混ぜていた。静的地図が返す本質的な情報は、指定waypoint・姿勢・
clearanceにおけるcollision-freeな横方向の連結区間である。

`recovery_footprint`へ全連結区間を返すAPIを追加し、controller cacheは次だけをkeyにする。

- normalized waypoint id
- heading bucket
- clearance bucket
- sample step

scalar course bounds全体を一度scanし、各Missionのlocal trust regionとpreferred offsetはcache
取得後に連結区間へ交差・選択する。heading bucket中心との差は車体外接半径から求めたguardで
footprintを拡張し、bucket内の任意headingを包含する。

## 2. Tracking MPC target-bound authority

MPCC trajectory生成は相手の時系列footprintを含めて毎周期検証する。Pass latch後かつ現在車体・
予測sweepが非重複なら、その検証済みtarget trajectoryを低レベルMPCが追従すればよい。
同じ相手境界をtracking MPCのhard state boundへ重ねると、狭いboundの揺れでOSQP反復が増える。

したがって次をすべて満たす間だけ、tracking MPCはphysical wall boundを使う。

- phase=Pass
- lateral clearance latch済み
- locked target継続、position jumpなし
- 現在車体非重複
- 予測sweep有効かつ非重複
- execution corridor非blocked
- EmergencyBrakeでない

条件を外れた周期は従来のtarget hard bound確認gateへ戻る。確認中も上位optimizer内の相手制約、
actual footprint guard、SafetyBrakeは有効であり、これらは変更しない。

## 3. Telemetry

既存の`Control callback runtime`と同じ1秒窓へ、cache request/hit/miss/evaluation、scan pose数、
entry数を出す。試走ではhit率とcallback時間を同じ窓で照合できるようにする。
