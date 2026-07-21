# Tasklist

- [x] 最新3車停止ログとChatGPT Proレビューを現行コードへ照合する。
- [x] 変更範囲と非採用項目を確定する。
- [x] Supervisor SAFE_STOP再試行とescape距離許容を実装する。
- [x] 追い越し横分離閾値を分割する。
- [x] 衝突後deliberate stop overrideを実装する。
- [x] moving V2X rollout clearanceを実装する。
- [x] 単体テストと正本仕様を更新する。
- [x] 対象パッケージのテストとビルドを実行する。

## 検証結果

- `make autoware-build`: 25 package成功。
- `colcon test --packages-select multi_purpose_mpc_ros`: 22 test target、563 test、失敗0。
- `mpc_controller_cpp`を変更後configで5秒起動し、`aggressive_sim=true`、
  `collision_stop_override=true`、`escape_distance_tolerance=0.10 m`の読込を確認した。
  終了code 124は意図した`timeout 5s`による終了である。

## 非採用

- `v2x_overtake_active_gap_loss_hold_sec`の0.4秒化は、WP61-63で確認済みのgap dropoutを
  再発させるため本変更では行わない。
- `coordinated_stop_front_speed_mps`の単純な0.5 m/s化は境界を移すだけなので、
  rollout separationを速度非依存で評価する。
