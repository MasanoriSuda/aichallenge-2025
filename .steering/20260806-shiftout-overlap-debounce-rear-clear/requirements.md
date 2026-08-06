# Requirements

## 目的

`output/20260806-115819` で確認した、追い越し実行中の二つの損失を解消する。

- 検証済み ShiftOut の predicted footprint overlap が周期ごとに反転し、前車速度capが短時間に再適用・解除される。
- locked targetを既に後方へ送った後の一時的な観測消失が、追い越し完遂ではなくRecoveryになる。

## 制約

- 追い越しentry条件とpre-armは変更しない。
- current body overlap、壁接触、target position jump、solver failureは従来どおりhard guardとする。
- predicted-overlap猶予は、完全mission検証済みで一度capを解除したShiftOutに限定する。
- target消失時の完遂は、最後に確認したcourse progressがreturn-clear距離を超えて後方であり、短いtarget hold中にrear-clear確認が成立した場合だけ許可する。
- ROS 2 topic/service契約は変更しない。

## 完了条件

- 短時間のpredicted overlapではShiftOutの解除済みfront capを保持する。
- predicted overlapが設定時間継続した場合はfront capを再適用する。
- current body overlapまたは壁接触では猶予しない。
- 最後にrear-clearを観測したtargetの短時間消失はReturnへ遷移できる。
- rear-clear未達のtarget消失は従来どおりHold後にRecoveryとする。
- core unit testとpackage buildが成功する。
