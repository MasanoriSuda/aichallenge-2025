# Design

## 1. Recovery履歴の早期終了

既存の`AdaptiveReverseRetryTracker`と`RecoveryIncidentLedger`は同じ
`adaptive_retry_reset_forward_distance_m`を使用する。この設定を5.0 mから2.0 mへ
変更する。Rejoin完了後にSupervisorがNormalで、正の実速度から2.0 mを積算した場合だけ
両履歴を解除するため、停止中やReverse中に誤ってfresh incidentにはならない。

## 2. Rolling再計画時の接触追跡継続

0.4 m境界では短区間primitiveだけを解除するが、rolling mission自体は継続する。
runtime contact判定でも`recovery_rolling_stepwise_reverse_active_`を改善追跡条件に含め、
初期パッチ内固定ではなく、前周期パッチと連結した非増加の移動を許可する。

これにより再計画境界で接触パッチがmap cellを跨いだだけの`new_contact`を防ぐ。

## 3. 接触悪化の連続確認

pure coreに`RecoveryCollisionWorseningGate`を追加する。

- rolling Reverse以外: raw判定を即時通過
- pose jump等のhard fault: 即時通過
- rolling Reverseの`NewContact`/`ContactWorsened`: 0.20秒連続でtrueの場合だけ通過
- raw判定が一度解消: pendingを即reset

ゲートは接触を安全と再分類するものではなく、短いoccupancy-cellチャタリングだけを
Supervisorから遮蔽する。持続悪化は従来どおり`StopBeforeDrive`へ遷移する。

## 影響範囲

- `stuck_recovery_core.hpp/.cpp`: 連続確認ゲート
- `mpc_controller_cpp.cpp`: 設定読込、rolling接触追跡、ゲート適用・ログ
- `config.yaml`, `config_for_cloud.yaml`: 0.20秒確認、2.0 m履歴reset
- `test_stuck_recovery_core.cpp`: transient/persistent/hard fault契約
