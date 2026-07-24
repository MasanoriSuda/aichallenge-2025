# 追い越しRecovery急減速調整 要件

## 目的

dev2の移動中低速車追い越しシナリオで、OvertakeLineが安全条件により
Recoveryへ移行した際、専用速度上限3.0 m/sによって高速車が低速車より遅くなり、
車間を広げて再追い越しを困難にする現象を、設定変更だけで緩和できるか確認する。

## ベースライン

- 対象run: `output/20260724-231447`
- d1追い越し開始: 10回
- d1 Recovery移行: 11回
- 正常な`Pass -> Return -> Idle`: 0回
- 代表区間:
  - d1実速度: 8.125 -> 3.652 m/s
  - d2実速度: 約5.4 m/s
  - 最小実加速度: -2.484 m/s2
  - Recovery中速度上限: 3.0 m/s
- V2Xは17.42 Hz、最大gap 71.5 msであり、通信欠損は主因ではない。

## 変更範囲

- `multi_purpose_mpc_ros/config/config.yaml`
  - `v2x_overtake_recovery_velocity`だけを3.0から5.0 m/sへ変更する。
- 本ステアリング文書と検証結果。

## 対象外

- C++ / Pythonソースロジックの変更
- wall margin、static wall、横加速度、EmergencyBrakeの緩和
- Recovery速度制限の無効化
- 再試行cooldown、side選択、corridor判定の同時調整
- 5 m Follow境界の調整
- ROS 2 topic/service/message、Domain、評価schemaの変更
- `output/`、rosbagなど生成物の編集

## 機能・安全要件

1. Recoveryの速度制限は有効なまま維持する。
2. Recovery中の目標上限だけを5.0 m/sへ変更し、壁・車体・横加速度の
   fail-closed条件は維持する。
3. 同じdev2シナリオを2周以上走行し、d1/d2双方の5トピックrosbagを取得する。
4. 制御指令、実速度、実加速度、V2X車間、OvertakeLine遷移を同一時刻で比較する。
5. 壁接触、SafetyStop、Stuck Recovery実行など新しい安全退行があれば不採用とする。

## Definition of Done

- 対象設定1項目以外に参加者制御差分がない。
- `make autoware-build`が成功する。
- dev2で2周以上のMCAPとAutowareログを取得する。
- ベースラインと以下を比較して記録する。
  - Recovery回数と理由
  - Recoveryごとの最低command/actual speed
  - 最小実加速度
  - d1がd2より遅い時間と車間拡大量
  - 追い越し完了回数
  - wall/contact/stuckの有無
  - lap time
- 改善・不変・悪化を分け、設定採用可否を結論づける。
