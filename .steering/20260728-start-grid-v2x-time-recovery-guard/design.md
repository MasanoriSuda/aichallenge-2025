# Start Grid V2X Time / Recovery Guard Design

作成日: 2026-07-28
状態: Complete

## 原因

### V2X receipt age

`V2XGapPlanner`はcallbackでROS時刻をreceipt時刻として保存する。一方、制御周期は周期冒頭で
取得したROS時刻を同じ周期のV2X評価へ渡す。MultiThreadedExecutor上で、その間にV2X callbackが
実行されると、最新receipt時刻が制御周期のsnapshotより数十ms先になる。

従来の`age_sec < 0.0`判定はこの正常な同時実行順序までvehicle dropoutとして扱い、
front-hazard holdを通じてSafetyBrakeを発生させた。

`v2x_overtake_core::is_v2x_receipt_age_fresh`を追加し、有限かつ
`-future_tolerance <= age <= timeout`をfreshとする。現行source timestampと同じ0.05秒を
receipt future toleranceとして使う。これは古いsampleのtimeout、ID/sample完全性、位置jumpを
緩和しない。

### Coordinated-stop Recovery

coordinated-stop入口は、前方停止車、Follow/SafetyBrake、設定速度以下だけを見ており、
Start-gridの正常な停止を除外していなかった。

`start_grid_grace::should_suppress_coordinated_recovery`を追加し、次のいずれかが真なら新規候補を
抑止する。

- Start-grid grace中
- Start-grid動的観測中
- Start-grid breakout継続中

開始済みRecovery episodeは従来のFSMと安全条件で完了させる。抑止対象は新規
`coordinated_stop_candidate`だけであり、衝突、壁証拠、solver fallbackなど他経路は変更しない。

## 影響

- ROS topic、message型、service、launch、提出物構造は変更しない。
- `expected_v2x_vehicle_count`の完全一致契約は維持する。
- 実車向けの速度・操舵・制動値は変更しない。
- SIM用Recoveryの開始条件だけをStart-grid中に限定する。

## 検証

1. pure helper単体テスト
2. `multi_purpose_mpc_ros`対象テスト
3. `make autoware-build`
4. 可能なら`make dev3`で、Start後にReverse要求がなくP1が正速度へ移ることを確認
