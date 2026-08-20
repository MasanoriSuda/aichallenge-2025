# Design

## 観測した欠陥

最新試走ではactive overtakeのwall hold自体は1周期で終わり、前回のMission失効修正は動作した。一方、11回のwall invalidation後に同じtarget/sideを短時間で再評価できるため、物理的に失敗した側を再び採る余地が残る。

またsolver-handoffは安全な予測を2周期連続で要求する。最新試走では予測のvalid/invalidが交互になる区間で最大40周期、約1秒の減速holdになった。

ログの `planner_min` はFrenet回廊境界から横目標までの余裕であり、実行時の `physical_min` は車体footprintから占有格子壁までの距離である。両者を `planner/physical wall distance` として直接比較する表示は診断上不正確である。

## 修正方針

### 1. 物理壁失敗をside retry blockへ接続

active ShiftOut / Passの予測経路を物理壁判定で棄却した時点で、既存の `PhysicalOrCommittedFailure` として同一target・同一sideのretry blockをarmする。

DynamicMissionWait中でも、現在Missionが物理的にinvalidatedされている場合は失敗側のrolling-prefix例外でcooldownを迂回させない。反対側は通常どおり候補評価できる。

### 2. solver wall handoffを1 fresh-safe observationへ短縮

solverがbounded continuationから復帰した周期には、通常MPCが新しく生成した予測経路を物理footprintで評価している。このsolver-handoffに限り、1回のfresh safe observationで制御を戻す。

予測がunsafe/unavailableの場合は回数に関係なくholdを維持する。active overtakeの監視条件は変更しない。

### 3. wall contractログを意味別にする

planner側を `frenet-corridor-reserve`、実行側を `physical-footprint-distance` として明記する。差分フラグも「同じ物理距離の不一致」ではなく、「採用済み計画契約に対して実行予測が不成立」というexecution contract mismatchへ改名する。

Mission invalidationログには、retry blockをarmしたか、反対側探索が許可されるかを残す。

## 影響範囲

- `mpc_controller_cpp.cpp`: retry feedback、solver handoff設定、決定ログ
- `overtake_execution_orchestrator.*`: wall契約telemetryの意味整理
- orchestrator unit tests

ROS interfaceとconfig schemaは変更しない。
