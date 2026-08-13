# Requirements

## 目的

`output/20260813-232215` で確認した、約22 m前方から不完全な追い越しMissionへコミットし、`rear_clear=inf` のままPassを継続して失速・Recoveryへ至る経路を止める。

## 要件

- 前方車の監視距離24 mと追い越し候補の計画距離30 mは維持する。
- 新規の横方向追い越しMissionへ実行権限を渡すのは、前方距離15 m以内とする。
- 15 mより遠方では候補評価、MPCC-lite shadow、ベースライン上の速度準備だけを許可する。
- FSMとMPCC-liteで、実行可能Missionの定義を共通化する。
- `rear-clear` の時刻または距離が未確認・不成立・非有限の候補は、新規追い越しを開始できない。
- 既にコミット済みのMission、SafetyBrakeからの再開、スタートグリッド処理は新規entry距離判定で中断しない。
- 壁、車体重複、EmergencyBrakeなどのhard guardは緩和しない。

## 変更範囲

- `multi_purpose_mpc_ros` の追い越しentry admission、MPCC-lite authority、設定、単体テスト。
- ROS topic/service/message契約は変更しない。

