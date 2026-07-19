# Design

## Decision

通常走行用OSQP MPCは正の経路進捗と通常速度を前提にしているため、Reverseへ流用しない。
Recovery専用に、符号付き移動距離を使うFrenet kinematic bicycle modelと離散操舵beam searchを置く。
これは毎周期第一入力だけを採用して再計画する、離散・有限ホライズンMPCである。

## Model

状態は`e_y`、`e_psi`、入力はtire angle `delta`とする。短い距離`dl`ごとに以下を予測する。

```text
e_y'   = e_y + dl * sin(e_psi)
e_psi' = e_psi + dl * (tan(delta) / L
                       - kappa_ref * cos(e_psi) / (1 - kappa_ref * e_y))
```

Forwardは`dl > 0`、Reverseは`dl < 0`。横偏差、姿勢差、操舵量、操舵変化と終端誤差を
評価し、beam内の最小cost列を選ぶ。

## Integration

- `LowSpeedRejoin`: 毎周期Recovery MPCを解き、第一tire angleを既存の操舵rate上限で制限する。
- `CheckClearance`: MPCの第一tire angleに近いStraight/Left/Rightと中間操舵候補を順位付けする。
- contact escape: contact reductionを第一優先し、同値候補だけMPCで順位付けする。
- static/V2X評価は従来の`recovery_footprint`をそのまま通す。
- planner invalid/no candidate時は従来のheading選択とrejoin P feedbackへfallbackする。

## Bounded invariants

- Reverse total distance: 3.0 m
- Reverse duration: 4.0 s
- Reverse speed: 0.8 m/s
- Escape target: 2.0 m
- Step: 0.40 m x max 10
- Attempts: max 3
- Rejoin timeout: 5.0 s

## Rollback

`stuck_recovery.recovery_mpc.enabled: false`でplannerを無効化し、元のbounded Recoveryへ戻す。
