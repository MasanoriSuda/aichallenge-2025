# Design

## 局所リファクタリング

新規追い越しentryの判定を、真偽値だけを返す`new_overtake_entry_speed_gate_allows`から、
次の二つを返すpure resolutionへ分離する。

- `execution_allowed`: 今周期にShiftOut/Passへhandoffしてよいか
- `prearm_active`: 完全Missionは成立しているが、実測速度条件だけが未達か

これにより「完全Mission成立」を速度ゲートの迂回条件ではなく、pre-armの開始条件として扱う。
広いFSM再編や新しいROS stateは追加しない。

## Pre-arm

pre-arm中のBehavior表現は`Follow`のままとし、OvertakeLineは`Idle`を維持する。一方で、
当該周期で再検証済みのMissionに限りgeneric Follow速度cap、follow gap planner、follow prepositionを
抑制し、基準走行線上で`target speed + ShiftOut最大closing speed`を上限とする縦加速を許可する。

実測相対速度が`v2x_overtake_entry_min_relative_speed`以上の状態を
`v2x_overtake_entry_speed_confirm_sec`連続確認した次の周期に、最新MissionをShiftOut/Passへ渡す。
設定値は「少し遅くても許可」の-0.5 m/sから、明確な前進余力を要求する+0.3 m/sへ変更する。

## 優先順位

1. EmergencyBrake、target discontinuity、Mission/corridor不成立
2. start-grid breakout専用handoff
3. committed ShiftOut/Pass/FollowPrepare継続
4. 新規entryの実測速度確認
5. 完全Mission成立中のpre-arm
6. 通常Follow

pre-armは安全判定を成立させるものではなく、同周期で既に成立したMissionの速度準備だけを担当する。

## ログ

既存の周期デバッグログへ次を追加する。

- pre-arm active
- entry relative speed
- entry stable duration
- pre-arm target velocity

状態変化時のreasonにもpre-armを明記し、ログ量は既存周期のまま増やさない。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure admission resolution
- `mpc_controller_cpp.cpp`: pre-armの速度・横ownership接続
- `config/config.yaml`: 実測相対速度閾値
- `test/test_v2x_overtake_core.cpp`: admission/pre-arm回帰試験
- `docs/spec/mpc-integration.md`: entry ownership仕様

ROS 2 interfaceおよび`aichallenge_system/`は変更しない。
