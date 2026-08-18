# Requirements

## 目的

最新走行で確認した、Frenet-DP execution prefix の絶対寿命切れごとに MPCC 実行権限が途切れる問題を解消する。

## 要求

- 同一 Mission generation・target・side の最新 QP 解だけを実行 prefix 候補にする。
- 候補は現在の壁形状、車体 footprint、既存 execution reference との差を再検証する。
- 採用は path 全体を一括で置換し、部分更新を行わない。
- 同じ solved source を再採用して実行期限を延命しない。
- 壁接触、壁 margin 不成立、EmergencyBrake、target discontinuity などの hard guard は緩和しない。
- 更新頻度は既存 rolling refresh interval で制限し、40 Hz callback ごとの重い再検証を避ける。
- 既存の ROS 2 topic/service、launch、提出インターフェースは変更しない。

## 対象外

- MPC solver failure 自体の解消。
- Recovery / Reverse の再設計。
- 車両寸法、壁 margin、追い越し admission parameter の攻撃化。
