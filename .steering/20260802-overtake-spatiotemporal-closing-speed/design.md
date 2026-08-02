# Design

## 現状

現行は左右、ShiftOut距離、corridor内横目標を複数評価している。一方、
body-clear deadlineは全候補で
`target_speed + v2x_overtake_shiftout_max_closing_speed`を使用している。
候補採用後もadaptive closing speedが設定の最小・最大値から再計算されるため、
候補評価時の速度仮定がmissionへ保存されない。

## Closing-speed候補

既存設定から次を重複なしで生成する。

```text
minimum
(minimum + maximum) / 2
maximum
```

現在設定が0.8～2.0 m/sなら、0.8、1.4、2.0 m/sを評価する。
新しいチューニングパラメータは追加しない。

## 候補評価

各横経路候補についてclosing speed候補を走査し、次を計算する。

```text
predicted_ego_speed = min(v_max, target_speed + closing_speed)
```

この速度をbody-clear deadline評価へ渡す。deadline成立候補を優先し、同条件なら
body-clearが早い候補を優先する。deadlineが無効、または時刻が同一の場合は、
既存の横経路順位を維持したうえで、より高いclosing speedを選ぶ。

全候補がdeadlineを外す場合も、現行の競技用soft policyを維持し、最も早く
body-clearする物理実行可能候補を返す。

## Mission固定

選択結果を次へ伝播する。

```text
SideAssessment
  -> V2XBehaviorOutput.overtake_selected_closing_speed
  -> OvertakeLineState.mission_closing_speed_limit
```

ShiftOutとfront-cap未解除Passでは、このmission値を設定上限の代わりに使う。
adaptive closing speedおよびunseparated closing reserveは、mission値より低い
速度を要求できる。したがって固定するのは最低速度ではなく、選択した攻撃度の
上限である。

Passのfront-cap解除後は従来どおり通常のレース速度参照へ移行する。

## ShiftOut Behavior ownership

body-clear deadline成立済みの固定経路をShiftOut中に実行している場合、entry用の
gap、curve、候補品質の再評価だけではBehaviorをFollowへ戻さない。

ownershipは次の条件で解除する。

- locked target消失または位置ジャンプ
- course progress不成立
- locked targetのpass side侵入
- explicit forbidden waypoint
- emergency front risk
- solver recovery要求

壁、live corridor、横加速度は従来どおりOvertakeLine実行層で検査する。
deadline不成立のsoft候補にはShiftOut ownershipを与えず、従来の再評価を残す。

## 非対象

- 追い越し専用MPC weight
- filtered course-relative target prediction
- lateral TTCおよびswitchback
- emergency・map・target continuity・solver hard guardの緩和
- acceleration上限の変更
