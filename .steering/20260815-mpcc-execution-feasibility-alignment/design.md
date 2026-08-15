# Design

## 1. hard corridorとsoft reserveを分離

DP sampleに次の二つを持たせる。

- hard interval: 実行時に物理的に成立しなければならない範囲
- preferred interval: robust clearanceを満たすことが望ましい範囲

target-active区間ではpass sideに応じて、target側へ物理target separation、壁側へ
hard wall clearanceを適用する。preferred intervalにはrobust target separationと
robust wall clearanceを適用する。preferred intervalが消滅してもhard intervalが
残るならMissionは候補に残し、DP costだけを増やす。

これにより「計画は通すが実行時hard gateで即座に落とす」を防ぎつつ、狭い区間を
全面禁止にしない。

## 2. tactical hysteresis

前回のFrenet DP pathがfreshかつ同一target/sideの場合、前回のtactical strategyも
requestへ渡す。別strategyへ切り替える候補には設定可能なpenaltyを加える。

前strategyがinfeasibleならpenaltyに関係なく別strategyを選択できるため、固定化ではなく
チャタリング抑制になる。

## 3. RecoveryRetention loopの終端

Recoveryを正常完了してFollowPrepareへMissionを保持した直後にruntime hard faultが
残っている場合、同じMissionを再度Recoveryへ戻さない。Mission retentionを禁止して
Idleへ終端し、次周期以降に新しい左右候補として評価し直す。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: corridor sample、DP cost、純粋判定
- `mpc_controller_cpp.cpp`: corridor構築、前戦術cache、RecoveryRetention終端
- `config.yaml`, `config_for_cloud.yaml`: soft reserveとstrategy switchの重み
- `test_v2x_overtake_core.cpp`: 単体テスト

topic/service/message/launch契約には変更なし。
