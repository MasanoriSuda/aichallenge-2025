# Design

## 現象

候補探索は動的corridor、static wall、横加速度を確認しているが、
「その横経路で車体横離隔がいつ成立するか」を順位付けに使っていない。
短いShiftOutでも横目標が小さい候補を選ぶと、前後距離がhard distanceへ
到達した時点で`body_clear=0`となり、SafetyBrakeへ移行する。

## Body-clear deadline

候補のsmoothstep ShiftOut経路を距離方向へサンプルし、targetの観測横速度を
短時間外挿する。車体中心の横距離が設定済みfootprint clearanceへ初めて
到達する時刻・走行距離を求める。

同時に、ShiftOut速度方針が許す最大closing speedを使い、現在の前後距離が
`v2x_moving_follow_hard_distance`へ到達する時刻を求める。

以下を満たす候補だけを実行候補とする。

```text
body_clear_time + deadline_margin <= hard_distance_time
```

既に横離隔済みならdeadlineは即成立とする。closingしていない場合、
hard-distance時刻は無限大として扱う。

## 候補順位

1. body-clear deadline成立
2. body-clear成立時刻が早い
3. direct pass
4. ShiftOut距離が短い
5. 横移動量が小さい
6. 必要横加速度が小さい

## Fail-safe

全候補がdeadline不成立ならOvertakeへ入らずFollowを継続する。実行開始後の
SafetyBrake、壁接触、動的corridor、MPC制約は従来どおり有効である。

