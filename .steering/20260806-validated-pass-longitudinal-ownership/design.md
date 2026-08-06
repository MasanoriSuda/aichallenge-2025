# Design

## 方針

追い越しPlannerが検証したMissionと、Behaviorの縦速度仲裁を別々の判定にしない。完全Missionの成立を「縦速度ownershipの引き渡し条件」として利用する。

## 1. 新規entry

次を全て満たす候補を validated entry とする。

- attack mode 有効
- Overtake要求、gap/zone成立
- selected Mission が feasible
- body-clear deadline が checked かつ feasible
- rear-clear prediction が checked かつ feasible
- front risk が EmergencyBrake ではない

validated entry は通常の0.3秒相対速度確認を待たず、同周期で Overtakeへ遷移する。この周期は generic Follow、front-risk、front-decel の速度制限を重ねず、MissionのShiftOut stage-speedへownershipを渡す。

## 2. committed ShiftOut

固定corridorと完全Missionを持ち、body-clear deadlineが成立し、current body footprintsが分離している間は、locked targetへの予測交差だけを理由にSafetyBrakeへ落とさない。

current footprint overlap、target jump/loss、inter-vehicle corridor、壁・経路・solver guardは緩和しない。

## 3. Pass境界揺れ

cap解放済みPassのcurrent-overlap確認を0.05秒から0.30秒へ変更する。これはhold-onlyであり、capの初回解放には使わない。0.30秒継続すれば従来どおりcap再適用・SafetyBrakeが可能になる。

## 4. SafeSeparationとReverse

SafeSeparationでtargetが再び明確に前方となった場合、Recovery速度で戻さず、同じ側のFollowPrepareへ移して現時点のcorridor再検証を待つ。

また、validated entryまたはcurrent-body-clearなcommitted executionが残るSafetyBrake/Followは、衝突通知がない限りcoordinated-stop由来のReverse候補にしない。衝突通知、wall evidence、solver fallbackからの復帰は従来どおり許可する。

## 非対象

- 衝突ペナルティ中のAWSIM速度固定
- 壁接触Recovery
- SafeSeparationのabsolute hard limit
- 車両寸法・wall marginの緩和

これらは本変更の効果確認後に別事象として扱う。
