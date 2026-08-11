# Design

## ギア・ハンドシェイク

`resolve_forward_overtake_handoff()` が求めたactionに対して、pending中のDrive要求をpure helperで仲裁する。

1. Drive要求前は既存どおり、Reverse走行中なら `HoldStop` とする。
2. 停止後は `RequestDrive` を発行する。
3. freshなDrive報告がない間は、0.25秒間隔で `RequestDrive` を再発行する。
4. 再要求間隔中は `HoldStop` を維持する。
5. freshなDrive報告を得た後だけpendingを解除し、既存のMission再検証結果に従う。

過去の `last_commanded_recovery_gear_` は通信上の実状態ではないため、要求抑止条件には使わない。

## 影響範囲

- `stuck_recovery_core.hpp/.cpp`: pending中のpure仲裁関数
- `mpc_controller_cpp.cpp`: 再要求時刻管理、古い指令履歴による抑止の撤去
- `test_stuck_recovery_core.cpp`: 停止前・再要求待ち・再要求期限・Drive報告の境界テスト

## 非対象

- 追い越しcandidate生成やside選択
- Stuck検出条件
- 壁・接触・solverのhard fault判定
