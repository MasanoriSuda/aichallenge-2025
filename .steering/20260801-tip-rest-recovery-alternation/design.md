# Design

## 1. 脱出距離の方向所有

Recovery状態が`REVERSE_MANEUVER`または`FORWARD_MANEUVER`なら、その状態を
脱出方向の正本とする。ギア切替・停止確認中は、直前に実際に開始したmaneuver方向を
保持して使用する。安全評価が次候補を反対方向へ変更しても、過去の移動距離を
新候補の短い閾値へ流用しない。

距離は現在maneuverの実測距離を使用する。既存のepisode距離は総移動量上限には
引き続き使用するが、脱出成功判定には使用しない。

## 2. 効かないForwardの方向交替

`ForwardDurationLimit`からaggressive retryへ入った回数を連続カウントする。
設定回数（初期値2）へ達したら、次のbounded candidate選択をReverse-onlyにする。

Reverseが実際に選択された後、またはReverseのcourse progressが悪化した場合は
強制を解除し、通常の候補比較へ戻す。これにより無限Forward固定を避けつつ、
Reverse固定の別の無限ループも作らない。

## 3. 影響範囲

- `stuck_recovery_core.hpp/.cpp`: 方向解決と候補方針の純粋関数
- `mpc_controller_cpp.cpp`: 実駆動方向の保持、再試行カウンタ、候補ゲート
- `config.yaml`: Forward連続失敗回数
- `test_stuck_recovery_core.cpp`: 回帰テスト

V2X契約、ギアtopic、最終制御出力の型・名称は変更しない。

