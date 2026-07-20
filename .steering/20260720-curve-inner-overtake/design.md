# Design

## 方針

既存の`curve_inner_pass_side()`が返す曲率内側符号をそのままイン差し側として使う。
操舵は既存OvertakeLineのShiftOut / Pass / Returnを使い、別制御器は追加しない。

pure coreへ内側カーブ追い越し判定を追加する。

- `entry_allowed`: 新規追い越し、soft curve、hard curve未到達、内側gap成立
- `hard_continuation_allowed`: ShiftOut/Pass継続中、hard curve、locked target観測、同じ内側gap継続

明示WP禁止、cooldown、EmergencyBrakeでは両方ともfalseとする。

## 側選択

イン差し進入が有効で未ロックの場合、soft curveでは内側をpreferred sideにする。
内側gapが不成立なら既存`try_both_sides`により外側を評価する。開始後は従来どおりlocked sideを最優先し、
旋回中に左右を切り替えない。

## 状態遷移への統合

`entry_allowed`の場合だけ、内側についてstart-curve、completion、soft-curve入口guardを緩和する。
hard curve内の新規開始は許可しない。開始済みラインは`hard_continuation_allowed`の間だけ維持し、
gapまたはtargetが消えた場合は既存continuity判定からRecoveryへ移す。

## 設定

- `v2x_overtake_inner_curve_entry_enabled: true`
- `v2x_overtake_inner_curve_hard_continuation_enabled: true`

構造体既定値とキー省略時はfalseとし、2025 AWSIM dev3比較実験だけで有効化する。

## 非対象

- hard curve内からの新規イン差し
- trajectory自体の変更
- gap膨張幅、SafetyBrake閾値、速度capの再調整
- 実車設定
