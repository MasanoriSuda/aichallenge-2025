# Design

## 1. Pass 速度 cap のヒステリシス

解除閾値は既存どおり 1.50 m とする。一度この閾値を満たした Pass に限り、横離隔が 1.30 m 以上なら解除状態を維持する。1.30 m 未満では cap を再適用する。

このヒステリシスは速度 cap のみに適用する。前方衝突保護から対象車を除外する判定は、引き続き現在の実横離隔 1.50 m を要求する。

## 2. corridor goal と対象車横離隔

固定 corridor center をそのまま目標にせず、現在の wall feasible bounds と対象車からの最低横離隔の共通範囲を求める。共通範囲がある場合は、その範囲内で固定 goal に最も近い点を使用する。

共通範囲がない場合は既存 wall bounds を優先し、速度 cap と壁・衝突保護を維持する。車両間 corridor と validated start-grid breakout は既に車両 inflation 済みのため対象外とする。

## 3. Pass 完了時の Return 優先

物理接触は従来どおり Recovery を最優先する。物理接触がなく、Pass で横離隔を一度確立済み、かつ対象車が 0.5 m 以上後方なら、壁余裕違反または static clamp Recovery の直前に Return へ移行する。

## 4. ログ

速度 cap の Released / Reapplied の変化時だけ、現在横離隔、解除閾値、再適用閾値、対象車縦距離を出す。周期ログは増やさない。
