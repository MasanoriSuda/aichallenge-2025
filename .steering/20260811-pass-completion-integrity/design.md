# Design

## 1. Latched forward escape

`pass_forward_completion_latched` を単独で速度維持の根拠にはしない。次を満たす場合だけ latched continuation を有効化する。

- SafeSeparation中かつPass committed
- target continuityあり
- 現在車体が非重複
- footprint predictionが有効
- execution corridorが非閉塞
- wall/EmergencyBrake/solver/forbidden waypointのhard faultなし
- rear-clear未成立

この判定はpure core関数にし、SafeSeparationの `forward_escape_allowed` と prediction-only overlap bridge の双方へ接続する。既存のlocal/absolute Pass budgetは維持する。

## 2. 横速度を含む到達可能領域

現在の到達可能横位置を、

`e_y(t) = e_y(0) + v_lat(0) * t + 0.5 * a_lat * t^2`

で評価する。`v_lat(0) = speed * sin(e_psi)` とし、目標が範囲外なら加速度上限上の点へ射影する。静的壁clamp後も同じ式で再検証する。

Returnも実行経路として同じwall/到達性判定を適用する。これにより、外向き運動を持つ実車体から非現実的なReturnを開始しない。

## 3. 高速停止車用の長距離ShiftOut候補

`v2x_overtake_line_max_shift_distance` を追加する。通常の4 m候補は残し、最大値まで段階的に候補を追加する。横加速度上限は6 m/s^2のままにし、長い距離を使って成立する候補だけを採用する。

静的Mission検証長も同じ最大ShiftOut距離を使い、候補生成と検証horizonの不整合を作らない。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure判定と横到達性
- `mpc_controller_cpp.cpp`: 状態接続、横速度入力、候補距離、Return実行検証
- `config/config.yaml`: 最大ShiftOut距離
- `test/test_v2x_overtake_core.cpp`: hard faultと横速度を含む境界テスト
