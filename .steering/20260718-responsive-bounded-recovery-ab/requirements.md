# Requirements

## Purpose

dev3のシミュレーション予選を対象に、強制再試行を使わない既存の段階的Recoveryへ戻し、
復帰の滑らかさを保ったまま応答だけを機敏にする。

## Scope

- `multi_purpose_mpc_ros/config/config.yaml`のsimulation-only Recovery設定。
- `make autoware-build`と対象package test。
- `make dev3`による3台走行とRecoveryログの比較。

## Constraints

- topic、service、message、Domain契約を変更しない。
- swept-footprint、V2X、距離、時間、step上限を緩和しない。
- `output/`を編集しない。
- 実車設定へ展開しない。

## Acceptance

- 積極再試行と強制rejoinが発火しない。
- Reverse時に実車速・移動距離が増加し、上限内で停止できる。
- Recovery発生時に`rejoin_complete`へ到達する、または安全ゲートで有限回停止する。
- 復帰後は通常MPC走行へ戻り、全車永久停止を回避する。
