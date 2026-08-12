# Requirements

## 目的

直近走行では、現在車体が非重複で新しい追い越し候補を生成できる状態でも、
Pass中の予測重複や将来wall clampを契機に

`Pass -> SafeSeparation -> FollowPrepare -> SafetyBrake -> Recovery`

へ移行している。既存のdynamic Mission waitは、横並び後のno-return状態では
同側再計画まで禁止するため、このケースで利用されていない。

実車体・壁・target continuityにhard faultがない場合は、失敗したMission世代を
再利用せず、現在位置から完全検証した同側Missionへ置換する。反対側への横断は
従来どおりno-return前だけに限定する。

## 要件

- `v2x_overtake_line_min_wall_clearance: 0.15`を維持する。
- 直線・外側の追加clearance 0.10 mを維持する。
- current bodyが非重複でtarget predictionが有効なら、no-return後もbounded dynamic
  Mission waitへ入れる。
- no-return後はfresh same-side Missionだけを採用し、反対側へ切り返さない。
- no-return前は既存のfresh alternate Mission選択を維持する。
- paused revalidationでは古いMissionのpredicted overlapをfresh candidateの棄却理由に
  使わず、候補自身のShiftOut/Pass/Return preflightを正とする。
- fresh candidateがある周期は、縦距離だけのEmergency判定で置換前にRecoveryへ
  落とさない。壁接触、車体重複、target discontinuity、solver異常はhard faultを維持する。
- 置換後もMission全体の既存時間・距離budgetを引き継ぐ。
- fresh candidateが得られなければ既存の短いwait期限後にRecoveryへ移る。

## Definition of Done

- core判定にno-return後のsame-side waitとalternate禁止の単体テストがある。
- controllerがfresh same-side candidateをatomicに置換できる。
- hard faultのfail-closed挙動が変わらない。
- 対象packageの単体テストとbuildが成功する。
- 動的確認はユーザー試走で、dynamic wait entry、same-side replacement、Recovery、
  SafetyBrake、Pass完遂率を前走と比較する。
