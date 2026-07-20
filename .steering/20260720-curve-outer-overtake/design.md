# Design

## 方針

既存の`curve_inner_pass_side()`が返す曲率内側符号の反対側を外側とする。
外まくりは既存のOvertakeLineを使い、別の操舵器やtrajectoryは追加しない。

pure coreへ次の2判定を追加する。

- `entry_allowed`: 新規追い越し、soft curve、hard curve未到達、外側gap成立
- `hard_continuation_allowed`: ShiftOut/Pass継続中、hard curve、locked target観測、外側gap継続

どちらも明示WP禁止、cooldown、EmergencyBrake、内側選択ではfalseとする。

## 状態遷移への統合

`entry_allowed`の場合だけ、従来の「カーブ手前3 m」「hard curve前に追い越し完了」の
入口guardを外側に限って緩和する。通常直線、内側、gap不成立には適用しない。

開始後のsoft curveは既存`continuing_overtake_allowed`で維持する。hard curveでは
`hard_continuation_allowed`を既存active-Pass継続条件へ加え、外側ShiftOutも継続可能にする。
外側gapまたはlocked targetが消えた場合は緩和せず、既存Continuity/Recoveryを使う。

## 設定

- `v2x_overtake_outer_curve_entry_enabled: true`
- `v2x_overtake_outer_curve_hard_continuation_enabled: true`

構造体既定値とキー省略時はfalseとし、2025 AWSIM dev3向けの比較実験だけで有効化する。

## 非対象

- イン差し
- trajectory自体の変更
- 壁・車両膨張幅、gap幅、速度capの再調整
- 実車設定
