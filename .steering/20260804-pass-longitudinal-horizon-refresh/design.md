# Design

## 方針

Pass mission の更新を次の 2 種類に分ける。

1. **Same-side geometric extension**
   - target の横位置変化に応じ、同じ side 内で横目標を調整できる。
   - 既存の `pass_horizon_max_extensions` を上限とする。
2. **Longitudinal horizon refresh**
   - 横目標を一切変更しない。
   - 現在の実速度、target 速度、最新 V2X prediction から rear-clear 必要距離を再計算する。
   - 同じ固定横目標について、延長後の Pass + Return 全経路を静的再検証する。
   - 32 m / 10 s の既存絶対上限を越えない。

## 状態遷移

`resolve_pass_horizon_action()` に `RequestLongitudinalRefresh` を追加する。

- rear-clear 済み、Return corridor 有効: `Return`
- 絶対上限到達または短期安全不成立: `Abort`
- 再検証不要: `Keep`
- 幾何延長回数に余裕あり: `RequestSameSideExtension`
- 幾何延長上限到達、かつ縦方向更新可能: `RequestLongitudinalRefresh`
- 最新の静的経路範囲内で fresh prediction だけ更新された: `Keep`
- 更新不能で残り horizon が不足: `EnterHold`

## Controller 実装

既存の horizon extension planner をモード化する。

- geometric mode:
  - same-side の feasible goal を再解決する。
  - 横調整上限を適用する。
  - `mission_extension_count` を増やす。
- longitudinal mode:
  - `fixed_pass_corridor_goal_ey` をそのまま使用する。
  - 許容横調整量を 0 として atomic commit する。
  - `mission_extension_count` は増やさず、診断用の
    `mission_longitudinal_refresh_count` のみ増やす。

両モードとも以下を再評価する。

- 現在実速度を初期値にした kinematic rollout
- rear-clear 必要距離
- 壁、横加速度、outer-role reversal を含む full-path static preflight
- target / side / generation / prediction expiry の atomic commit 条件

## 安全境界

- target ID または side が変わった場合は commit しない。
- 固定横目標を wall clamp する必要がある場合は refresh しない。
- 実 footprint 非分離、wall contact、EmergencyBrake は Pass 継続条件にしない。
- 絶対上限は既存の 32 m / 10 s を維持する。

## 効果確認ログ

- `same-side Pass horizon extended`
- `Pass longitudinal horizon refreshed`
- `Pass -> Return`
- `SafeSeparation entered`

次回走行では、縦 refresh 回数、Pass 完遂数、SafeSeparation 移行理由を比較する。
