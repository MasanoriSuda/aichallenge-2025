# Design

## 方針

solver失敗後の操舵を次の3モードに分ける。

1. `hold`: 最初の設定周期は直近の可解操舵を保持する。
2. `path-track`: 通常走行では参照曲率、横偏差、方位偏差から退避操舵を計算する。
3. `neutralize`: Overtake Recoveryまたは参照値が無効な場合だけ中立へ戻す。

`path-track` の操舵目標は、既存の空間誤差feedbackを再利用し、現在速度と
`ay_max`、操舵出力gainから横加速度上限を適用する。現在操舵から目標操舵への
遷移は `steer_rate_max` で制限する。

## 局所リファクタリング

- 中立専用だったrate limiterを任意目標へ遷移できる共通関数へ変更する。
- 「中立化開始」の判定を「直近操舵hold解除」の判定へ改名し、制御意図を明確にする。
- path-track目標計算をMPC内部helperへ分離する。

## 非対象

- ShiftOutの横オーバーシュート
- Reverse／stuck recoveryの調整
- 追い越し候補の左右選択
