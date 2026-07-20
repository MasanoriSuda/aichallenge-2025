# 設計

## 背景

位置計測の遅れを `t_delay`、車速を `v` とすると、MPC が受け取る位置の概算遅れは `v * t_delay` になる。`0.3 s` の場合、20 km/h で約 1.67 m、40 km/h で約 3.33 m となり、高速域ほど軌跡とのずれが顕著になる。

一方、trajectory の解像度が約 `0.6 m` のとき `wp_id_offset=2` は約 `1.2 m` の固定先読みである。速度によって時間換算が変わり、計測時刻そのものも補正しないため、自己位置遅延の代用にはしない。

## 変更方針

### EKF

`reference.launch.xml` に次の任意引数を追加する。

- `simulation_pose_additional_delay`: 既定 `0.3 s`
- `vehicle_pose_additional_delay`: 既定 `0.0 s`

既存の `simulation` 引数で選択し、選択値を `ekf_localizer` の `pose_additional_delay` に渡す。

### MPC

2025 Pure Pursuit はEKFの `0.3 s` 補正に加え、受信位置を速度とヨーレートで `0.125 s` 先へ円弧予測していた。現行MPCにも、MPC初期状態を作る直前に同等のCTRV予測を追加する。

- `state_prediction_delay_sec`: 既定 `0.125 s`、`0.0 s` で無効
- `state_prediction_simulation_only`: 既定 `true`
- 直進近似と円弧解をヨーレートで切り替える
- 受信 `Odometry` は変更せず、値オブジェクトに予測結果を格納する
- 非有限値は既存のodometry fail-safeで拒否する

この予測はEKFの計測遅延補正後に、制御計算・アクチュエータ遅延を補うためのものである。`wp_id_offset` は参照先読みであり、自己位置予測には使わない。現在の C++ 実装では非ゼロoffset時に、基準 waypoint と先読み waypoint の間で spatial state と参照フレームがずれる可能性があるため、両 offset は `0` のままとする。

### 検証

まずCTRV予測の直進・旋回・後退・入力検証を単体テストし、XMLとビルドを確認する。走行試験では同じ trajectory・速度設定でEKF補正とMPC予測の値を比較し、次を確認する。

- 高速コーナーでの横偏差と進路の膨らみ
- D1 で継続していた OSQP max-iteration の発生頻度
- 操舵の位相遅れ、蛇行、コーナー内側への過補正の有無

過補正する場合は、まずMPCの `state_prediction_delay_sec` を `0.125` から `0.05`〜`0.1 s` へ下げる。それでも残る場合にEKFの追加遅延を `0.3` から `0.1`〜`0.2 s` へ下げる。

## 互換性

topic/service/message 名は変更しない。EKFとMPC初期状態予測はいずれも実車では既定無効とし、シミュレーションだけに適用する。
