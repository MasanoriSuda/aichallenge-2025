# Design

## Paired world

最初のruntime NPC教師試走で、NPCがAWSIM native V2X fanoutへ参加しないことを確認
した。単車empty publisherを止めてもMPCは`NoData`となるため、このrunを教師として
採用しない。

代わりに公式固定gridの3 vehicle worldを用い、domain 2を既存configの15 km/h低速peer、
domain 3を学習対象egoとする監査pairを作った。全domainでLiDAR/IMU/GNSS/V2X
infrastructureを起動するが、学生nodeの唯一のsensor subscriptionは`/scan`である。

動的試走ではdomain 3だけでなく、周回後に低速domain 2へ追いついたdomain 1も
ShiftOut中のwall-margin違反からEmergency Stop/Recoveryへ波及した。このためこのpairを
教師生成経路へ昇格せず、`e2e-peer-audit-*`として明示的に監査用途へ降格する。

## Admission

教師候補runはFinish、LiDAR / command topic、finite command、衝突・停止・Recoveryを
監査する。衝突後の回復や長時間停止を含むrunは、教師が出力を発行していても学習へ
自動採用しない。今回のpeer runはこのadmissionで不合格であり、datasetは生成しない。

学生 candidate は次の順で評価する。

1. offline run-level validation
2. `e2e-single` 3 laps regression
3. `e2e-npc-single` runtime NPC closed loop
4. 二つ目の seed は最初の gate が成立した後に追加

## Longitudinal limitation

現baselineはsteering imitation + fixed accelerationである。runtime NPC baselineでは
停止後も+0.6 m/s2を指令し続けたため、横回避だけでなくLiDARまたは許可されたwheel
odometryに基づくlongitudinal safety policyが必要である。fixed accelerationの閾値調整
ではなく、別Sliceで入力・教師・停止条件を明示して構成する。
