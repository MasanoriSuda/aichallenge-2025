# Design

## 1. Pre-arm physical-wall cache warming

`OvertakeLine` が Idle で、pre-arm 中の complete Mission candidate がある場合に限り、candidate の frozen ShiftOut / Pass / Return path から horizon の横位置と向きを生成する。

1 control cycle あたりの評価 stage 数を設定値で制限し、物理壁 envelope cache を段階的に埋める。active Mission の optimizer と同じ waypoint、heading、clearance bucket を使い、追い越し開始時の全 horizon cold scan を分散する。

これは経路を採用・変更する処理ではなく cache warming のみであり、Mission authority や安全判定には影響しない。

## 2. Cold-load-aware RTI refinement

MPCC の第 1 solve 後、次のいずれかなら第 2 RTI solve を実行しない。

- active Mission の開始から設定時間以内
- `init_problem()` 中の物理壁 envelope cache miss 数が設定閾値以上

第 1 feasible 解は従来どおり採用する。skip は `SkipColdLoad` として既存の condition/deadline skip から区別して記録する。

## 3. Scope

- 変更: `mpcc_progress` の refinement decision、MPC problem metadata、prewarm helper、設定と telemetry、unit test
- 非変更: V2X target selection、左右戦術、wall/vehicle clearance、Recovery、ROS topic/service 契約
