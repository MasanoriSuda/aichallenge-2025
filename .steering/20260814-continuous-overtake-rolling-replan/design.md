# Design

## 1. Rolling replan context

`FollowPrepare + DynamicMissionWait`をrolling replan contextとして扱う。これは新しいMission entryではなく、
実行中Missionのsoft failure後に次prefixを探索する短い継続状態である。

## 2. MPCC-lite authority

- shadow evaluation対象へpaused Missionを含める。
- rolling replan contextではprogressive prefix admissionをactive executionとして実行する。
- 同側prefixはno-return後でも接続可能とする。
- 反対側prefixは従来どおりno-return前かつtarget/body/predictionが有効な場合だけ許可する。
- invalidated generationのCurrentSideHold候補はunavailableとし、freshなLeft/Right候補を選ばせる。
- current pathが失効済みでfresh prefixが得られた場合、score差や通常のreplacement間隔を待たず置換する。

## 3. Behavior ownership

rolling replan contextでtarget continuityと物理条件が有効な間はBehaviorをOvertakeに保つ。これにより、
entry用のFollow capやCruiseへの一時遷移を挟まず、次prefixへhandoffできる。

## 4. Physical hold line

fresh prefix評価中は現在の`e_y`を基準に、各horizon点を壁内へclampしたhold lineを出力する。
このlineも壁・横加速度のexecution checkを通し、成立しなければ従来のRecoveryへ移る。
速度は現在速度を急に捨てず、target速度との差を既存の未分離closing上限内に保持する。

## 5. 終了

fresh prefixが得られればtransactional replacementでShiftOut/Passへ戻る。得られない場合は既存の
dynamic wait time/distance limitでMissionを終了する。hard faultは待たずに従来処理へ渡す。
