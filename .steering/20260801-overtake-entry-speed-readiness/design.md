# Design

## Scope

今回の変更は新規Overtake admissionだけに限定する。最新事故で確認された並走中Recoveryは次段の
独立作業とし、本変更には含めない。

## Entry speed readiness

監視対象は、LineがIdleで、Behaviorが新規Overtake候補を評価している前方または側方targetとする。
対象速度が有限で、次を満たす間だけready開始時刻を保持する。

```text
ego_speed - target_speed >= entry_min_relative_speed
```

対象ID変更、速度不足、無効値、対象消失では開始時刻を破棄する。連続時間が
`entry_speed_confirm_sec`以上になったときだけreadyとする。

## Final common gate

BehaviorがOvertakeを要求しても、次の全条件に該当する場合は新規入口とみなし、speed readyを必須にする。

```text
requested == Overtake
Line is not ShiftOut / Pass / FollowPrepare
previous Behavior is not Overtake
```

未成立時は最終BehaviorをFollowへ変更する。既にLineが所有するmission、およびOvertake Behaviorを
Lineへ渡す1周期のhandoffは継続させる。

## Parameters

- `v2x_overtake_entry_min_relative_speed: -0.5`
- `v2x_overtake_entry_speed_confirm_sec: 0.3`

攻撃性を残すため、小さな初期速度劣勢は許容する。一方、最新事故の約-1.7 m/sは確実に棄却する。

## Diagnostics

遮断中は既存V2X debugの`desired=Overtake, final=Follow`とともに、reasonへ次を含める。

- target ID
- relative speed
- required minimum
- stable / required duration

新しい周期ログは追加しない。

