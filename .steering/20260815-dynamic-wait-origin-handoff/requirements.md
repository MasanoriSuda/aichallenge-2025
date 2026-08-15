# Requirements

## 背景

`output/20260815-090846` では、追い越し候補の target-bound 不成立後に
`ShiftOut -> FollowPrepare` へ移り、短い再選択期限を越えても同一 Mission を
最大 15 秒保持する事象が 3 回確認された。未完了の ShiftOut と、既に横分離を
完了した Pass 相当の実行を同じ DynamicMissionWait として扱っていることが、
追従化と Mission budget expiry の主経路になっている。

## 要求

- 横分離を完了した ShiftOut の target-only 制約不成立では、壁・車体・緊急制動の
  hard fault がなければ、現在の物理的に成立した前進 prefix を保持して再計画する。
- 横分離未完了かつ no-return 前の ShiftOut 由来 DynamicMissionWait は、短い
  `dynamic_mission_wait_*` 期限で終了し、Recovery を挟まず新規左右探索へ戻す。
- Pass または no-return 後の DynamicMissionWait は、物理的に安全な前進 prefix
  の権限がある間だけ rear-clear 待ちを許す。
- 壁接触、壁余裕違反、車体の非回復 overlap、EmergencyBrake、solver Recovery、
  forbidden waypoint は従来どおり fail-closed とする。
- ROS 2 topic/service、launch、評価結果 schema、設定値は変更しない。

## 対象外

- MPCC の目的関数・horizon パラメータ変更
- Recovery / Reverse の調整
- 車体寸法・壁クリアランスの変更
