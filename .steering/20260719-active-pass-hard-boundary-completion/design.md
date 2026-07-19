# Design

## 方針

`v2x_overtake_active_hard_curve_completion_enabled`を追加する。既定は`false`、今回のdev3
A/Bだけ`true`とする。

開始済みOvertakeかつOvertakeLineが`Pass`のとき、次のhard境界までの距離を既存
`distance_to_hard_overtake_boundary`で求める。境界までの加速可能距離から
`sqrt(v^2 + 2 * a_max * distance)`で到達可能速度を見積もり、trajectory参照速度と
global/domain cap以下へ制限する。

pure coreのactive completion policyは既存`resolve_pass_completion`を利用する。ただし
ShiftOutは完了済みなのでshift距離は0、Pass側のA/B rear-clearは0.5 m、hard境界bufferは
0.5 m、merge-back距離は要求しない。これはPass側のまま境界へ入るシミュレーション予選
向けの攻めた設定であり、実車向け値ではない。

## 拒否条件

- 新規passまたはOvertakeLineがShiftOut/Return/Recovery/Idle
- 明示禁止WP
- hard境界が未検出、または現在位置が境界buffer内
- 相対速度不足、またはavailable distance < required distance
- curve cooldown、EmergencyBrake
- 後段のgap/locked side/wall判定不成立

## 診断

V2X debugと状態遷移へ`hard_continue`、hard境界距離、available/required distanceを出す。

## 互換性

topic/service/messageは変更しない。新規yaml keyを省略した場合は`false`で従来挙動を維持する。
