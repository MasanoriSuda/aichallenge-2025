# Design

## 1. Side selection

次カーブ内側の探索距離を通常の curve-forbidden 判定から分離する。直線でも設定距離内で最大曲率の符号から内側を解決する。

minimum-motion候補に以下を渡す。

- required lateral shift
- base racing line clear
- corridor width
- continuous open distance

選択順は以下とする。

1. 現在のレーシングラインを維持できる候補
2. 十分な幅・連続距離を持つ次カーブ内側（外側との差が許容横移動量以内）
3. 必要横移動量が最小の候補

片側だけ実行可能な場合は従来どおりその側を選ぶ。

## 2. Active-pass SafetyBrake arbitration

前後距離由来の SafetyBrake を抑制できる条件を純粋関数で判定する。

- phase が ShiftOut または Pass
- nearest front が locked target
- fixed validated corridor が存在し、inter-vehicle corridor ではない
- target を継続観測し、position jump がない
- 現在車体footprintが非重複
- footprint prediction が有効で、予測sweepも非重複

成立時はlocked targetについてのみ、front emergency / hard-center SafetyBrakeをRelativeSpeedLimit相当へ降格する。別車両と不確実なケースはfail-closedを維持する。

既存のOvertakeLine壁・横加速度・実行可能性判定は変更しない。

## 3. Diagnostics

V2X debugへ以下を追加する。

- lookahead inner side
- inside preference eligibility
- committed corridor front-danger suppression

起動ログへ新設定値を出す。

## 4. Verification

- core unit tests
- `make autoware-build`
- `colcon test --packages-select multi_purpose_mpc_ros`
- `git diff --check`
- 実走では side selection、SafetyBrake回数、Pass->Return成功数、壁Recoveryを比較する。

