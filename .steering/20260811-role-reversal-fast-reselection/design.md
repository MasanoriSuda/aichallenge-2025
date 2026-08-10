# Design

## 1. Role-reversal reserve

現行の course-role 評価は runtime continuation reserve を含むが、実走では役割反転が
評価終端付近にあり、初期計画時は `role_reversal_s=inf`、実行時は
`outer pass becomes inside before rear-clear` となった。

初期値を 6 m とし、predicted rear-clear 後の曲率反転を確実に観測する。これにより、
以下の候補は entry 時に棄却または preflight 済み transition 付き候補へ限定される。

```text
entry role = outer
rear-clear + reserve role = inner
scheduled full-track transition = unavailable
```

## 2. SafeSeparation tactical reselection

SafeSeparation 中でも、次を全て満たす場合だけ fresh alternate candidate の即時再選択を
許可する。

- forward escape が成立していない
- target continuity が有効
- current body と predicted sweep が非重複
- target が 4 m 以上前方
- execution corridor が blocked でない
- rear-clear 未成立
- 壁・EmergencyBrake・solver の hard fault がない

候補 debounce 未完了でも、last-feasible cache が fresh かつ motion-fresh であり、現在 pose
から full preflight が再成功した場合だけ transactional replacement を許可する。接触中や
side-by-side 中の横断は許可しない。

## 3. Fast dynamic-wait release

通常の FollowPrepare は 4 秒/20 m を維持する。runtime continuation failure から入った
dynamic Mission wait だけは 0.75 秒/4 m で終了する。fresh current/alternate Mission が
その間に成立すれば既存の atomic replacement を使い、成立しなければ failed side を
retry-block して Idle へ戻し、次周期から両側を比較する。

## 4. Safety boundary

即時再選択は physical separation と fresh prediction がある場合だけで、actual overlap、
wall contact/margin violation、EmergencyBrake、solver recovery では起動しない。速度制限、
車体 footprint、壁 clearance は変更しない。
