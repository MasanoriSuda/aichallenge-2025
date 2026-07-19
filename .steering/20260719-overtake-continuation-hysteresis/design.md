# Design

## 方針

`v2x_overtake_guard_min_front_distance` を新規開始用として残し、
`v2x_overtake_continue_min_front_distance` を追加する。

V2X behavior FSM がすでに `Overtake` の周期では継続閾値を選択し、通常 gap guard と
fallback guard の両方へ同じ phase policy を渡す。継続中は開始準備距離を再判定しないが、
gap 幅と横移動到達性は再評価する。

設定省略時は継続閾値を開始閾値へフォールバックさせ、既存 yaml の意味を変えない。
継続閾値が開始閾値を超えないよう読み込み時に clamp する。

## 影響範囲

- `config/config.yaml`
- `src/mpc_controller_cpp.cpp`
- `v2x_overtake_core` と unit test
- `docs/spec/mpc-integration.md`

ROS 2 topic/service/message と評価インターフェースは変更しない。
