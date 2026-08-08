# Design

## 1. Target-scoped pre-arm validation lease

entry speed readinessのidentityを`target + side + closing speed`から`target`へ狭める。
実測相対速度の確認は横候補に依存しないため、同一targetに対する最新の完全Missionが毎周期
再検証されている限り、左右やclosing speedが変わっても履歴を継続する。

完全Missionが一時的に欠落した場合は、同一targetかつhard guardが健全な場合だけ、短い
`validation_hold_sec`の間、速度確認の内部履歴を維持する。この周期は通常Followのままとし、
pre-armの加速ownershipもShiftOut handoffも許可しない。

handoff条件は次の全てとする。

1. 今周期の完全Missionがvalidated
2. target-scoped実測相対速度が確認済み
3. hard guardが健全
4. pre-arm総時間・距離budget内

## 2. Full Mission owns completion admission

粗いcompletion-distance guardは、完全Mission候補を作る前の早期判定として残す。
一方、ShiftOut/Pass/Returnのbody-clear、rear-clear、壁、Returnを検証済みの候補は、
その候補自身をentry completionの正本とし、次hard-curveまでの単一距離比較で再棄却しない。

これはhard curveを無条件に許可する変更ではない。候補のfull-mission preflight、現在の
gap/curve policy、Emergency、禁止waypoint、cooldownは従来どおり必須とする。

## 3. Observability

既存の周期ログへ次を追加する。

- `prearm_lease`: validated Mission欠落中のbounded履歴保持
- `prearm_lease_remaining`
- `completion_mission_override`: 完全Missionが粗いcompletion guardを置換したか

既存ログ周期を変えず、イベントログの大量追加は行わない。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure lease resolverとcompletion permission拡張
- `mpc_controller_cpp.cpp`: target-scoped履歴、hard guard、最新Mission handoff
- `config/config.yaml`: validation hold時間
- `test/test_v2x_overtake_core.cpp`: leaseとcompletionの回帰試験
- `docs/spec/mpc-integration.md`: entry ownershipの更新

ROS 2 interfaceと`aichallenge_system/`は変更しない。
