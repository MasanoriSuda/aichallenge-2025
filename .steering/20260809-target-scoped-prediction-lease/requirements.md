# Requirements

## 目的

停止車向けOvertake entryとPass prediction leaseの責務境界を明確にし、別targetの停止証拠、汎用Abort理由、prediction欠損中の攻撃速度が誤って再利用されないようにする。

## 必須要件

- 停止確認証拠は現在のMission target IDと一致する場合だけentry-speed確認を省略できる。
- 停止判定速度は`low_speed_avoidance_max_front_speed`と一致させる。
- prediction leaseはtyped reasonが`TargetPredictionUnavailable`の場合だけ開始できる。
- target discontinuity、course progress rejection、corridor block、wall/body fault、absolute budgetはleaseできない。
- lease中は通常Pass attack速度を所有させず、lease開始時速度を超える正の加速を要求しない。
- fresh prediction復帰後は通常Pass速度へ戻る。
- 壁、EmergencyBrake、solver recovery、外部Reverse後の古いMission破棄は従来どおり維持する。

## 制約

- ROS 2 topic、message、launch、評価インターフェースは変更しない。
- `aichallenge_system`は変更しない。
- `aichallenge/result-summary.json`の既存ユーザー変更には触れない。

