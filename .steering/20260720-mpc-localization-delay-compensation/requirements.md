# MPC 自己位置遅延補正

## 目的

高速走行時に、MPC が参照する `/localization/kinematic_state` の時刻遅れによって、計画進路が外側へ膨らむ可能性を先に低減する。

## 要件

- シミュレーションでは EKF の `pose_additional_delay` を `0.3 s` にする。
- 実車では未計測の補正を適用せず、既定値を `0.0 s` にする。
- いずれの値も launch 引数で上書き可能にし、A/B 比較できるようにする。
- 昨年の Pure Pursuit で使っていた `0.125 s` の等速・等ヨーレート予測を、MPC初期状態にも適用する。
- MPC側の予測は受信odometryを変更せず、シミュレーション時だけ有効にする。
- MPC側の予測時間は設定可能とし、`0.0 s` で無効化できるようにする。
- `wp_id_offset` は MPC の制御遅延向け先読みであり、自己位置の計測遅延補正には使わない。
- 現在判明している tight curve での状態・参照フレーム不整合を避けるため、`wp_id_low_offset` と `wp_id_offset` は `0` を維持する。
- ROS 2 topic、message、service、Domain の既存契約を変更しない。

## 制約

- `0.3 s` は 2025 由来の既存調整値であり、2026 AWSIM の遅延を実測した確定値ではない。
- 走行ログに ground truth または時刻比較可能な rosbag がないため、今回の変更だけで厳密な遅延量は確定しない。
- 実車へはシミュレーション結果をそのまま展開しない。

## Definition of Done

- SIM/実車で異なる EKF 追加遅延を設定できる。
- SIM限定のMPC初期状態予測と単体テストが追加される。
- MPC 設定と仕様書に責務分離が記載される。
- launch XML の構文確認が通る。
- `make autoware-build` が通る。
