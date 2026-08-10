# Requirements

## 目的

追い越し中の候補更新失敗を、直ちに FollowPrepare / Recovery と車速低下へ波及させない。
直近に全 Mission preflight を通過した同側または反対側の候補を短時間保持し、
SafeSeparation の soft failure 時に現在の制御を止めず原子的に差し替える。

## 背景

`output/20260810-211117/d1/autoware.log` では、10回 Pass に入った一方で
Return 完了は1回だけだった。左右候補評価は周期的に実行されているが、その結果は
当該周期の `V2XBehaviorOutput` にしか残らず、SafeSeparation abort が別周期で発生すると
Missionを無効化して FollowPrepare / Recovery へ落ちている。

## 要求

- 直近の実行可能な同側候補と反対側候補を、target ID・Mission generation・評価時刻と共に保持する。
- 保持候補には短い有効期限を設ける。
- wall contact、EmergencyBrake、target discontinuity、solver recovery 等の hard fault では再利用しない。
- soft SafeSeparation abort時は、安定確認済みの反対側候補を優先し、なければ同側候補を使う。
- 反対側候補は従来の no-return と安定時間を満たした場合だけ再利用する。
- 同側候補は no-return 後でも利用できるが、実車体非重複とtarget continuityを必須とする。
- 候補がない・古い・差し替え失敗の場合は既存のdynamic wait / Recoveryへ戻す。
- topic、message、launch、提出インターフェースは変更しない。

## 非対象

- GMPCCへの全面置換
- planner別thread/process化
- 車両モデルや加速度上限の変更
- wall/contact hard guardの緩和

## Definition of Done

- pure policyの単体テストを追加する。
- controllerが候補cacheを更新・期限判定・soft abort rescueへ利用する。
- rescueの採用・棄却理由を低頻度イベントログで確認できる。
- `make autoware-build` 相当の対象package buildとunit testが成功する。
