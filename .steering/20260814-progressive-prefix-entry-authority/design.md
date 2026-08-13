# Design

## 1. Entry prefix admission

`MpccLitePrefixExecutionRequest`に`new_entry_context`を追加する。prefixは次のどちらかで評価する。

- active execution: 従来どおりno-return前のrolling replacement
- new entry: Idle、commit window内、hard faultなしの新規ShiftOut

new entryは開始前なのでno-return条件を満たすものとする。それ以外の物理条件とbudgetはactive時と
同じresolverで判定する。

## 2. Authorityと実行可能Missionの統一

MPCC-lite authorityは新規entryで、完全Missionまたはadmitted progressive prefixを選択できる。
選択されたsideを同一周期の`SideAssessment`へ反映し、progressive prefixを
`has_executable_mission`として扱う。これにより、zone判定、縦方向ownership、Behavior FSM、
OvertakeLineの順に同じ判断を伝播する。

## 3. Fresh-prefix lease

MPCC-liteの約7 Hz評価間隔でauthorityが消えないよう、同一target・同一sideの選択を短時間保持する。
ただし保存Missionをそのまま再実行せず、各周期に生成されたprogressive prefixへ物理admissionを
再適用する。current prefixがない、期限切れ、hard fault、commit window外なら実行権を失う。

## 4. 縦横の同時handoff

progressive prefixがadmittedされた場合だけ`validated_overtake_entry_longitudinal_owner`を有効にする。
相対速度準備中は既存pre-armを使用し、準備完了周期では同じfresh prefixを横Missionとして渡す。
横経路が消えた状態でpre-armだけを実行権として扱わない。

## 非対象

Pass後のSafeSeparation、Recovery、壁余裕値、制御周期、full MPCCへの置換は変更しない。
