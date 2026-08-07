# Design

## 1. Latched forward completion

初回認可では、予測 footprint、壁、target continuity に加えて rear-clear までの必要走行距離を確認する。初回距離 budget は SafeSeparation の通常窓と許可済み extension を合わせた有限値とする。

認可後は `OvertakeLineState` に latch を保持し、毎周期変化する必要距離や `target_s` 上限だけでは解除しない。実/確認済み footprint 重複、予測不成立、壁異常、pass-side intrusion、EmergencyBrake、solver recovery のいずれかで解除・Abortする。rear-clear 成立時は Return へ進む。

SafeSeparation には latch 状態を明示し、latched 中は `target clear ahead` の RecoverBehind 分岐を通さない。

## 2. Mission-scoped bounded pre-arm

pre-arm の同一性を次で判定する。

- target vehicle ID
- pass side sign
- selected Mission の closing speed（小さな数値揺れは許容）

完全な validated Mission が現在周期に存在するときだけ速度安定時間を蓄積する。Mission 消失または同一性変更で timer と走行距離をリセットする。

pre-arm には時間・走行距離上限と retry cooldown を設ける。上限到達時はその Mission の pre-arm を終了し、cooldown 後に新しい Mission として再評価する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure decision helpers
- `mpc_controller_cpp.cpp`: state, config parsing, orchestration, diagnostics
- `config/config.yaml`: bounded pre-arm defaults
- `test/test_v2x_overtake_core.cpp`: regression tests

ROS interface 変更なし。
