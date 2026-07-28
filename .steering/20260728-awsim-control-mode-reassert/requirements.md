# AWSIM Control Mode Reassert Requirements

作成日: 2026-07-28
状態: Complete

## 背景

提出環境のrosbag
`output/20260728-130100-release/rosbag2_autoware (1).mcap`では、開始直後に
`/control/command/control_cmd`が速度11.11 m/s、加速度+1.0 m/s²を継続していた一方、
`/localization/kinematic_state`の車速は全記録期間でほぼ0 m/sだった。

評価側`autostart_orchestrator`の`/awsim/control_mode_request_topic=true`は一度だけの
publishであり、ログ上のsuccessはAWSIMの受理確認ではない。提出環境でsubscriber discovery
またはAWSIM初期化が遅れるとAUTONOMOUS engage要求を取りこぼし、正常な制御指令と
Reverse指令の両方が適用されない可能性がある。

## 要求

1. 修正は提出物`aichallenge_submit/`内に閉じる。
2. `/awsim/control_mode_request_topic`の名前、型`std_msgs/msg/Bool`、`true`の意味を変えない。
3. SIMの`Ready`進入時にAUTONOMOUS engage要求を即時送信し、発進確認まで周期再送する。
4. SIMの`Start`進入時にも再送windowを再度開始し、実車速が設定閾値に達するか設定timeoutまで要求を周期再送する。
5. 発進確認中だけEvidence-free Stuck Recoveryの新規開始を抑止する。
6. timeout後および一度発進した後は、衝突・壁・solver failureを含む既存Recoveryを維持する。
7. 実車経路では再送処理を無効にする。
8. topicのsubscriber数が0でも再送を継続し、DDS discovery完了後の周期で届くようにする。

## Definition of Done

- 再送判定をROS非依存の単体テストで確認する。
- `multi_purpose_mpc_ros`がビルドできる。
- topic/service/Domain/提出tar構造を変更していない。
- 実走はユーザー側の提出環境で確認する。
